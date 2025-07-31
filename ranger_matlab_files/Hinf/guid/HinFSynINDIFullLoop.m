
clear all
%close all

sample_time = 1/500;

am = 0.025;
as = 0.05;
Amd=tf(am,[1 am-1],sample_time);
Asd=tf(as,[1 as-1],sample_time);
Amc=d2c(Amd,'tustin');
Asc=d2c(Asd,'tustin');

%% STABILIZATION LOOP DATA
Kps = [4.1089; 4.7304; 7.36];
Kds = [10; 18; 18];

Hs_cutoff_freq = 5;

[Hcs_num,Hcs_den] = butter(2,2*pi*Hs_cutoff_freq,'s');
Hcs = tf(Hcs_num,Hcs_den);
[Ahcs,Bhcs,Chcs,Dhcs] = ssdata(Hcs);
Ahs7 = [];
Bhs7 = [];
Chs7 = [];
Dhs7 = [];
Ahs3 = [];
Bhs3 = [];
Chs3 = [];
Dhs3 = [];


% Boucle pour créer le blkdiag 7 fois
for k = 1:7
    Ahs7 = blkdiag(Ahs7, Ahcs);
    Bhs7 = blkdiag(Bhs7, Bhcs);
    Chs7 = blkdiag(Chs7, Chcs);
    Dhs7 = blkdiag(Dhs7, Dhcs);
end
for k = 1:3
    Ahs3 = blkdiag(Ahs3, Ahcs);
    Bhs3 = blkdiag(Bhs3, Bhcs);
    Chs3 = blkdiag(Chs3, Chcs);
    Dhs3 = blkdiag(Dhs3, Dhcs);
end


Gs = [-10.67  10.67  0     0     0     0 0;
      8      8     -22.22 17.39 0     0 0;
      0      0      0     0     29.68 0 0;
     -0.847 -0.847 -0.847 0     0     0 0;
      0      0      0    -0.002 0     0 0]./1000;
Gs_inv = pinv(Gs);

Gs_thrust = zeros(1,7);
Gs_thrust = [0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            0 0 0 0 0 0 0;
            -0.847 -0.847 -0.847 0 0 0 0;
            0 0 0 -0.002 0 0 0]./1000;
        
%% GUIDANCE LOOP DATA 
Kpg = diag([0.6; 0.6]);
Kdg = diag([2.0; 1.8]);

Hg_cutoff_freq = 5;

[Hcg_num,Hcg_den] = butter(2,2*pi*Hg_cutoff_freq,'s');
Hcg = tf(Hcg_num,Hcg_den);
[Ahcg,Bhcg,Chcg,Dhcg] = ssdata(Hcg);
Ahg2 = [];
Bhg2 = [];
Chg2 = [];
Dhg2 = [];


% Boucle pour créer le blkdiag 3 fois
for k = 1:2
    Ahg2 = blkdiag(Ahg2, Ahcg);
    Bhg2 = blkdiag(Bhg2, Bhcg);
    Chg2 = blkdiag(Chg2, Chcg);
    Dhg2 = blkdiag(Dhg2, Dhcg);
end


lsp = [logspace(-3, 3, 500)];

% % 

% % %%%%%%%%%%%%%%%%%%% S


kw1=0.05;
numW1=[1 1.5];
denW1=[1 0.015];




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  KS 
kw2=0.1;%0.5
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

Kposx = tunableGain('Kposx',1,1);  % 1 outputs, 2 inputs
Kposz = tunableGain('Kposz',1,1);  % 2 outputs, 2 inputs

K=append(Kposx,Kposz);




%Kss = tunableTF('Ktf',nK-1,nK);

%%%%%%%%%%%%%%%%%% ATTENTION mettre 'UseParallel',false, si absence de PCT,
%%%%%%%%%%%%%%%%%% et eventuellument réduire RandomStart.

[At,Bt,Ct,Dt] = linmod('HINF4INDIFullLoop');

iconst=iconst+1;
sys_hinf{iconst} = minreal(ss(At,Bt,Ct,Dt));

sys_hinf{iconst}.InputName{1}='roll';
sys_hinf{iconst}.InputName{2}='pitch';
sys_hinf{iconst}.InputName{3}='yaw';
sys_hinf{iconst}.InputName{4}='posx';
sys_hinf{iconst}.InputName{5}='posz';


sys_hinf{iconst}.OutputName{1}='Mx';
sys_hinf{iconst}.OutputName{2}='My';
sys_hinf{iconst}.OutputName{3}='Mz';
sys_hinf{iconst}.OutputName{4}='Fx';
sys_hinf{iconst}.OutputName{5}='Fz';

sys_hinf{iconst}.OutputName{6}='eposxw';
sys_hinf{iconst}.OutputName{7}='eposzw';
sys_hinf{iconst}.OutputName{8}='uw1';
sys_hinf{iconst}.OutputName{9}='uw2';

% 



Nbspecif=length(sys_hinf);
Tbf=[];

for j= 1:(Nbspecif)%%%%%%%%%%%%%%%%% CONSTRUCTION DE M(s) 
    Ttemp=lft(sys_hinf{j},K,2,2); 
    Tbf=append(Ttemp,Tbf);
end;

T=Tbf;%%%%%%%%%%%%%%%%%%%%%% Le FAMEUX M SI pas de contrainte de stabilité sur K
CstrS1=1;
CstrS2=1;

ReqSposx =  TuningGoal.Gain('posx','eposxw',CstrS1); %    S  <= 1 (filtre deja  inclus)
ReqSposz = TuningGoal.Gain('posz','eposzw',CstrS2);       

%ReqK = TuningGoal.ControllerPoles('Kss',1e-11); %%%%%% 

CstrKS1=1;
ReqKS1 = TuningGoal.Gain('posx','uw1',1); %    KS  <= 1 
ReqKS2 = TuningGoal.Gain('posz','uw2',1); %    KS  <= 1 


option=systuneOptions('Display','iter','RandomStart',8,'MaxIter',500,'UseParallel',true,'MinDecay',0.1);
[CL, fSoft, gHard] = systune(T, [ReqSposx ReqKS1 ReqSposz ReqKS2],option); 
%K=ss(CL.Blocks.Kss);
% K=CL.Blocks.Ktf;
% [A,B,C,D]=tf2ss(K.numerator.Value,K.denominator.Value);
% K=ss(A,B,C,D);

%K=get(get(CL).Blocks.K).Gain.Value;
Kposx=ss(CL.Blocks.Kposx)
Kposz=ss(CL.Blocks.Kposz)
Kpg=blkdiag(Kposx.D,Kposz.D)

%%%%%%%%%%%%%%%%%%%%%% ANALYSE DES RESULTAS

[a, b, c, d] = linmod('CRIT4INDIFullLoop');
ss_t = ss(a, b, c, d);
ss_t=minreal(ss_t);



figure;
mod_inv_wS=sigma(invW1, lsp);
semilogx(lsp, 20*log10(mod_inv_wS),'r-',lsp, 20*log10(fSoft(1)*CstrS1*mod_inv_wS),'g-');
hold on;
mod_S = sigma(ss_t(6,4), lsp);
semilogx(lsp, 20*log10(mod_S),'b-');
xlabel('|S|(bleu), 1/|W1|(rouge), g/|W1|(vert)'); grid; hold on;
grid on;
zoom on;


figure;
mod_inv_wKS=sigma(invW2, lsp);
semilogx(lsp, 20*log10(mod_inv_wKS),'r-',lsp, 20*log10(fSoft(2)*mod_inv_wKS),'g-');
hold on;
mod_KS = sigma(ss_t(8,4), lsp);
semilogx(lsp, 20*log10(mod_KS),'b-');
xlabel('|KS|(bleu), 1/|W2|(rouge), g/|W2|(vert)'); grid; hold on;
grid on;
zoom on;




figure;
mod_inv_wS=sigma(invW1, lsp);
semilogx(lsp, 20*log10(mod_inv_wS),'r-',lsp, 20*log10(fSoft(3)*mod_inv_wS),'g-');
hold on;
mod_S = sigma(ss_t(7,5), lsp);
semilogx(lsp, 20*log10(mod_S),'b-');
xlabel('|S|(bleu), 1/|W1|(rouge), g/|W1|(vert)'); grid; hold on;
grid on;
zoom on;



figure;
mod_inv_wKS=sigma(invW2, lsp);
semilogx(lsp, 20*log10(mod_inv_wKS),'r-',lsp, 20*log10(fSoft(4)*mod_inv_wKS),'g-');
hold on;
mod_KS = sigma(ss_t(9,5), lsp);
semilogx(lsp, 20*log10(mod_KS),'b-');
xlabel('|KS|(bleu), 1/|W2|(rouge), g/|W2|(vert)'); grid; hold on;
grid on;
zoom on;



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

