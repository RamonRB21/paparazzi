
close all;
clear all;

%load('G2');%%%%%%%%%%%%%%%% modele
%load('INDIcr');
sample_time = 1/500;

Kp = [4.1089; 4.7304; 7.39];
Kd = [10; 18; 18];

am = 0.025;
as = 0.08;
Amd=tf(am,[1 am-1],sample_time);
Asd=tf(as,[1 as-1],sample_time);
Amc=d2c(Amd,'tustin');
Asc=d2c(Asd,'tustin');

H_cutoff_freq = 5;

[Hc_num,Hc_den] = butter(2,2*pi*H_cutoff_freq,'s');
Hc = tf(Hc_num,Hc_den);
[Ahc,Bhc,Chc,Dhc] = ssdata(Hc);
Ah7 = [];
Bh7 = [];
Ch7 = [];
Dh7 = [];
Ah3 = [];
Bh3 = [];
Ch3 = [];
Dh3 = [];


% Boucle pour créer le blkdiag 7 fois
for k = 1:7
    Ah7 = blkdiag(Ah7, Ahc);
    Bh7 = blkdiag(Bh7, Bhc);
    Ch7 = blkdiag(Ch7, Chc);
    Dh7 = blkdiag(Dh7, Dhc);
end
for k = 1:3
    Ah3 = blkdiag(Ah3, Ahc);
    Bh3 = blkdiag(Bh3, Bhc);
    Ch3 = blkdiag(Ch3, Chc);
    Dh3 = blkdiag(Dh3, Dhc);
end


Hd = c2d(Hc,sample_time,'tustin');
H_discr_num = cell2mat(Hd.num);
H_discr_den = cell2mat(Hd.den);

G = [-10.67  10.67  0     0     0     0 0;
      8      8     -22.22 15.62 0     0 0;
      0      0      0     0     26.67 0 0;
     -0.847 -0.847 -0.847 0     0     0 0;
      0      0      0    -0.002 0     0 0]./1000;
G_inv = pinv(G);

G_thrust = zeros(1,7);
G_thrust = [0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            -0.847 -0.847 -0.847 0 0 0 0;
            0 0 0 -0.002 0 0 0]./1000;

lsp = [logspace(-3, 3, 500)];

% % 

% % %%%%%%%%%%%%%%%%%%% S


kw1=0.2;
numW1=[1 20];
denW1=[1 0.01];




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  KS 
kw2=0.0005;%0.5
numW2=1;
denW2=1;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GS
% kw3=0.0001;
% %kw3=1;
% numW3=1;
% denW3=1;



%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% 



W1=kw1*tf(numW1,denW1);
W2=kw2*tf(numW2,denW2);
%W3=kw3*tf(numW3,denW3);




invW1=inv(W1);
invW2=inv(W2);



%nK=30;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
iconst=0;

Kroll = tunableGain('Kroll',1,1);  % 1 outputs, 2 inputs
Kpitch = tunableGain('Kpitch',1,1);  % 2 outputs, 2 inputs
Kyaw = tunableGain('Kyaw',1,1);  % 2 outputs, 2 inputs

K=append(Kroll,Kpitch,Kyaw);




%Kss = tunableTF('Ktf',nK-1,nK);

%%%%%%%%%%%%%%%%%% ATTENTION mettre 'UseParallel',false, si absence de PCT,
%%%%%%%%%%%%%%%%%% et eventuellument réduire RandomStart.

[At,Bt,Ct,Dt] = linmod('HINF4INDIstab');

iconst=iconst+1;
sys_hinf{iconst} = minreal(ss(At,Bt,Ct,Dt));

sys_hinf{iconst}.InputName{1}='roll';
sys_hinf{iconst}.InputName{2}='pitch';
sys_hinf{iconst}.InputName{3}='yaw';

sys_hinf{iconst}.OutputName{1}='Mx';
sys_hinf{iconst}.OutputName{2}='My';
sys_hinf{iconst}.OutputName{3}='Mz';
sys_hinf{iconst}.OutputName{4}='erollw';
sys_hinf{iconst}.OutputName{5}='epitchw';
sys_hinf{iconst}.OutputName{6}='eyaww';
sys_hinf{iconst}.OutputName{7}='uw1';
sys_hinf{iconst}.OutputName{8}='uw2';
sys_hinf{iconst}.OutputName{9}='uw3';

% 



Nbspecif=length(sys_hinf);
Tbf=[];

for j= 1:(Nbspecif)%%%%%%%%%%%%%%%%% CONSTRUCTION DE M(s) 
    Ttemp=lft(sys_hinf{j},K,3,3); 
    Tbf=append(Ttemp,Tbf);
end;

T=Tbf;%%%%%%%%%%%%%%%%%%%%%% Le FAMEUX M SI pas de contrainte de stabilité sur K
CstrS3=0.5;
CstrS1=1.2;

ReqSroll =  TuningGoal.Gain('roll','erollw',CstrS1); %    S  <= 1 (filtre deja  inclus)
ReqSpitch = TuningGoal.Gain('pitch','epitchw',1);       
ReqSyaw = TuningGoal.Gain('yaw','eyaww',CstrS3);        %    

%ReqK = TuningGoal.ControllerPoles('Kss',1e-11); %%%%%% 

CstrKS3=0.5;
ReqKS1 = TuningGoal.Gain('roll','uw1',1); %    KS  <= 1 
ReqKS2 = TuningGoal.Gain('pitch','uw2',1); %    KS  <= 1 
ReqKS3 = TuningGoal.Gain('yaw','uw3',CstrKS3); %    KS  <= 1 


option=systuneOptions('Display','iter','RandomStart',16,'MaxIter',500,'UseParallel',true,'MinDecay',1e-3);
[CL, fSoft, gHard] = systune(T, [ReqSroll ReqKS1 ReqSpitch ReqKS2 ReqSyaw ReqKS3],option); 
%K=ss(CL.Blocks.Kss);
% K=CL.Blocks.Ktf;
% [A,B,C,D]=tf2ss(K.numerator.Value,K.denominator.Value);
% K=ss(A,B,C,D);

%K=get(get(CL).Blocks.K).Gain.Value;
Kroll=ss(CL.Blocks.Kroll)
Kpitch=ss(CL.Blocks.Kpitch)
Kyaw=ss(CL.Blocks.Kyaw)
Kp=blkdiag(Kroll.D,Kpitch.D,Kyaw.D)

%%%%%%%%%%%%%%%%%%%%%% ANALYSE DES RESULTAS

[a, b, c, d] = linmod('CRIT4INDIstab');
ss_t = ss(a, b, c, d);

ss_t=minreal(ss_t);


figure;
mod_inv_wS=sigma(invW1, lsp);
semilogx(lsp, 20*log10(mod_inv_wS),'r-',lsp, 20*log10(fSoft(1)*CstrS1*mod_inv_wS),'g-');
hold on;
mod_S = sigma(ss_t(4,1), lsp);
semilogx(lsp, 20*log10(mod_S),'b-');
xlabel('|S|(bleu), 1/|W1|(rouge), g/|W1|(vert)'); grid; hold on;
grid on;
zoom on;


figure;
mod_inv_wKS=sigma(invW2, lsp);
semilogx(lsp, 20*log10(mod_inv_wKS),'r-',lsp, 20*log10(fSoft(2)*mod_inv_wKS),'g-');
hold on;
mod_KS = sigma(ss_t(7,1), lsp);
semilogx(lsp, 20*log10(mod_KS),'b-');
xlabel('|KS|(bleu), 1/|W2|(rouge), g/|W2|(vert)'); grid; hold on;
grid on;
zoom on;




figure;
mod_inv_wS=sigma(invW1, lsp);
semilogx(lsp, 20*log10(mod_inv_wS),'r-',lsp, 20*log10(fSoft(3)*mod_inv_wS),'g-');
hold on;
mod_S = sigma(ss_t(5,2), lsp);
semilogx(lsp, 20*log10(mod_S),'b-');
xlabel('|S|(bleu), 1/|W1|(rouge), g/|W1|(vert)'); grid; hold on;
grid on;
zoom on;



figure;
mod_inv_wKS=sigma(invW2, lsp);
semilogx(lsp, 20*log10(mod_inv_wKS),'r-',lsp, 20*log10(fSoft(4)*mod_inv_wKS),'g-');
hold on;
mod_KS = sigma(ss_t(8,2), lsp);
semilogx(lsp, 20*log10(mod_KS),'b-');
xlabel('|KS|(bleu), 1/|W2|(rouge), g/|W2|(vert)'); grid; hold on;
grid on;
zoom on;



figure;
mod_inv_wS=sigma(invW1, lsp);
semilogx(lsp, 20*log10(mod_inv_wS),'r-',lsp, 20*log10(fSoft(5)*CstrS3*mod_inv_wS),'g-');
hold on;
mod_S = sigma(ss_t(6,3), lsp);
semilogx(lsp, 20*log10(mod_S),'b-');
xlabel('|S|(bleu), 1/|W1|(rouge), g/|W1|(vert)'); grid; hold on;
grid on;
zoom on;



figure;
mod_inv_wKS=sigma(invW2, lsp);
semilogx(lsp, 20*log10(mod_inv_wKS),'r-',lsp, 20*log10(fSoft(6)*CstrKS3*mod_inv_wKS),'g-');
hold on;
mod_KS = sigma(ss_t(9,3), lsp);
semilogx(lsp, 20*log10(mod_KS),'b-');
xlabel('|KS|(bleu), 1/|W2|(rouge), g/|W2|(vert)'); grid; hold on;
grid on;
zoom on;

% 
% 
% 
% figure;
% mod_inv_wT=sigma(invW3, lsp);
% %mod_G=sigma(Gs, lsp);
% semilogx(lsp, 20*log10(mod_inv_wT),'r-',lsp, 20*log10(gfin*mod_inv_wT),'g-');
% hold on;
% mod_T = sigma(ss_t(3,1), lsp); 
% semilogx(lsp, 20*log10(mod_T),'b-');
% 
% xlabel('|T|(bleu), 1/|W3|(rouge), g/|W3|(vert)'); grid; hold on;
% grid on;
% zoom on;

