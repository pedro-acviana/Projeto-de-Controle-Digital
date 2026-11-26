% proj2cdig_m.m — Prj2CDig: PI por realimentação de estados + observador de perturbação senoidal
% ENE0167 Controle Digital · UnB · grupo 2
%
% Baseado no Prj2CDig_m.m do Prof. Adolfo (Referencias/Prj2CDig), adaptado para:
%   * usar o modelo P2DZ já identificado no Prj1 (proj1cdig/outputs/E/modelo_identificado.mat),
%     sem refazer a identificação;
%   * atuador em contagens de 16 bits (U_MAX = 21000), PO = 50 °C, w0 = 0,065 rad/s, Tr = 30 s;
%   * polos do controlador pela receita do roteiro (p1 cancelado por Nb, p2 sobre o zero, p3 = Tr);
%   * conferir Tr, Mp e a rejeição da senoide numa réplica LTI do Prj2CDig.slx antes do Simulink;
%   * rodar o Prj2CDig.slx (cópia nesta pasta) com o observador ligado e desligado.
%
% Rode seção por seção (Ctrl+Enter) a partir da pasta proj2cdig/.
% Fase 1 = projeto contínuo (passo 1 do roteiro). O discreto (Ts = Td) entra na Fase 2.
%
% Variáveis que o Prj2CDig.slx lê do workspace (NÃO renomear):
%   Kp Tp1 Tp2 A B (escalares da forma modal)  Ta PO w0  K(1:2) Ki Nb L(1:4)
%   x1i x2i (processo)  x1ei x2ei (observador)  xs1i xs2i (gerador)  xii (integrador)
%   AMP_W AMP_DC (perturbação; ligadas aos blocos Step1/Step2 pela seção 8)

clear; close all; clc

%% 0. Parâmetros  <<< TROQUE AQUI >>>
ARQ_MODELO = '../proj1cdig/outputs/E/modelo_identificado.mat';
ARQ_PRBS   = '../proj1cdig/outputs/E/proj1cdig_coleta_2026-11-12_02-49-21.csv';   % só para Ta

PO   = 50;          % ponto de operação [°C]
Tr   = 30;          % tempo de subida 10–90 % desejado [s], resposta de 1ª ordem (Mp = 0)
w0   = 0.065;       % frequência da perturbação senoidal [rad/s] (período 96,7 s)
umax = 21000;       % = U_MAX do firmware [contagens de 16 bits]
umin = 0;
U0_MEDIDO = 10200;  % u que deixa Tp estável em 50 °C na bancada

% Perturbação no Simulink: W entra no processo somada a Ta, em °C equivalentes na entrada
% (W constante = w °C desloca Tp em w °C em regime; equivale a w/Kp contagens de u).
% Ajustar após a Fase 0: se o cooler senoidal (ws) causa ±a °C em Tp em malha aberta,
% use AMP_W = a / |Gn(j·w0)|, com Gn = G/Kp (a seção 6 imprime |Gn(j·w0)| ≈ 0,31).
AMP_W  = 5;         % amplitude da senoide em W [°C] (1800 a 3000 s)  <<< ajustar na Fase 0
AMP_DC = 5;         % degrau constante de W a partir de 1000 s [°C]    <<< ajustar na Fase 0

OBS_FATOR = 12;     % polos do observador = OBS_FATOR × polos do processo (ponto de partida)
OBS_ON    = [1 0];  % simulações da seção 8: observador ligado e desligado
SALVAR_FIG = false; % true grava as figuras da seção 8 em outputs/fase1/

%% 1. Modelo identificado no Prj1 (PRBS de 12/11/2026)
%                    1 + Tz s
%   G(s) = Kp ------------------- e^(-Td s),   y = Tp − Ta,  u em contagens
%             (1 + Tp1 s)(1 + Tp2 s)
M = load(ARQ_MODELO, 'Kp', 'Tz', 'Tp1', 'Tp2', 'Td');   % o .mat também tem K/Ts do PI do Prj1: não carregar
Kp = M.Kp; Tz = M.Tz; Tp1 = M.Tp1; Tp2 = M.Tp2; Td = M.Td;
if Tp2 > Tp1, [Tp1, Tp2] = deal(Tp2, Tp1); end          % Tp1 = polo lento
G  = tf(Kp*[Tz 1], conv([Tp1 1], [Tp2 1]), 'InputDelay', Td);
fprintf('Modelo: Kp = %.7f °C/cont  Tz = %.2f s  Tp1 = %.2f s  Tp2 = %.2f s  Td = %.3f s\n', ...
        Kp, Tz, Tp1, Tp2, Td);

% Temperatura ambiente: média medida na coleta PRBS (entra no .slx como Ta constante)
Ta = 26.8;
if isfile(ARQ_PRBS)
    T  = readtable(ARQ_PRBS, 'TextType', 'string');
    Ta = mean(T.ta(T.modo == "PRBS"), 'omitnan');
end
fprintf('Ta = %.2f °C (média da coleta PRBS)\n', Ta);
fprintf('Modelo absoluto do .slx (Tp = Ta + Kp·u em regime): u no P.O. = %.0f cont. (medido: %d)\n', ...
        (PO - Ta)/Kp, U0_MEDIDO);

%% 2. Espaço de estados na forma modal (como o script do professor)
%   G(s)/Kp = A/(Tp1 s + 1) + B/(Tp2 s + 1)
%   x' = Ap x + Bp u,   y = Cp x,   Ap = diag(-1/Tp1, -1/Tp2),  Bp = [Kp; Kp],  Cp = [A/Tp1  B/Tp2]
% A e B (escalares) são lidos pelo .slx: as matrizes ficam com nomes Ap, Bp, Cp.
A = (Tz - Tp1)/(Tp2 - Tp1);
B = 1 - A;
Ap = [-1/Tp1 0; 0 -1/Tp2];  Bp = [Kp; Kp];  Cp = [A/Tp1  B/Tp2];
fprintf('\nForma modal: A = %.5f  B = %.5f\n', A, B);
erro_tf = norm(tf(ss(Ap, Bp, Cp, 0)) - tf(Kp*[Tz 1], conv([Tp1 1], [Tp2 1])), inf);
fprintf('  confere com G(s) sem atraso: ||diferença||∞ = %.1e\n', erro_tf);

%% 3. Controlador: realimentação de estados + canal PI (Nb na referência, Ki no erro)
%   Estado aumentado [x; xi], xi' = r − y.   u = Nb·r + Ki·xi − K·x
%   p1 = −2,2/(3 Tr): cancelado pelo zero do canal PI (Nb s + Ki), com Nb = −Ki/p1
%   p2 = −1/Tz      : sobre o zero do processo, cancela-se com ele
%   p3 = 3·p1 = −2,2/Tr: único polo que sobra → Y/R = 1/(τ s + 1), τ = Tr/2,2
Aa = [Ap      zeros(2,1);
      -Cp     0        ];
Ba = [Bp; 0];
p1 = -2.2/(3*Tr);
pc = [p1, -1/Tz, 3*p1];
Ka = acker(Aa, Ba, pc);
K  = Ka(1:2);
Ki = -Ka(3);
Nb = -Ki/p1;
fprintf('\nPolos do controlador: %s\n', mat2str(pc, 4));
fprintf('K  = [%.4f  %.3f]   (esperado ≈ [1.31 29.6])\n', K);
fprintf('Ki = %.3f            (esperado ≈ 36.7)\n', Ki);
fprintf('Nb = %.1f            (esperado ≈ 1503)\n', Nb);
fprintf('  u logo após degrau de r: %.0f cont./°C → degrau de 5 °C: u0 + %.0f\n', Nb, 5*Nb);

%% 4. Malha fechada r → y (contínua, sem atraso): Tr, Mp, ganho DC
Amf = Aa - Ba*Ka;
Bmf = [Bp*Nb; 1];
Tyr = ss(Amf, Bmf, [Cp 0], 0);
Tyr_min = minreal(zpk(Tyr), 1e-6);
S = stepinfo(Tyr);
fprintf('\nY/R após cancelamentos: %s', evalc('disp(Tyr_min)'));
fprintf('Tr = %.2f s (esp. %g s)   Mp = %.2f %% (esp. 0 %%)   ganho DC = %.4f   ts(2%%) = %.1f s\n', ...
        S.RiseTime, Tr, S.Overshoot, dcgain(Tyr), S.SettlingTime);
figure('Name', 'Degrau de referência (contínuo)');
step(Tyr, tf(1, [Tr/2.2 1]), '--', 150); grid on
legend('Y/R projetado', '1/(\tau s + 1), \tau = Tr/2,2', 'Location', 'southeast')
title(sprintf('Degrau unitário de r: Tr = %.1f s, Mp = %.1f %%', S.RiseTime, S.Overshoot))

%% 5. Observador aumentado: processo + gerador de senoide em w0
%   Gerador: xs' = As xs,  d = Cs xs = xs1 (perturbação na entrada do processo, em contagens)
%   Ao = [Ap  Bp·Cs; 0  As],  Co = [Cp 0 0]
As = [0 1; -w0^2 0];
Cs = [1 0];
Ao = [Ap          Bp*Cs;
      zeros(2)    As   ];
Bo = [Bp; 0; 0];
Co = [Cp 0 0];
fprintf('\nPosto da observabilidade de (Ao, Co): %d de %d\n', rank(obsv(Ao, Co)), size(Ao, 1));
po = OBS_FATOR*[-1/Tp1, -1/Tp2, -1/Tp1, -1/Tp1];
L  = acker(Ao', Co', po)';                     % polo triplo: o acker avisa "more than 10% in error",
                                                % mas os autovalores conferidos abaixo batem
fprintf('Polos do observador (%g× os do processo): %s\n', OBS_FATOR, mat2str(po, 4));
fprintf('L = [%s]''\n', sprintf(' %.5g', L));
fprintf('  polos de Ao − L·Co conferidos: %s\n', mat2str(sort(eig(Ao - L*Co)).', 4));

%% 6. Réplica LTI do Prj2CDig.slx: rejeição da senoide com observador ligado e desligado
% Igual ao diagrama: o processo recebe u + Ta + W/Kp (W e Ta somam na entrada, em °C); o observador
% recebe v = Nb r + Ki xi − K x̂ (antes de subtrair d̂) e NÃO tem o acoplamento Bp·Cs. Com u = v − d̂,
% a dinâmica do erro de estimação é exatamente Ao − L·Co (troca de sinal do estado do gerador).
% Estados: [x(2); xi; x̂(2); ŝ(2)]   entradas: [r; Ta; W]   saídas: [y; u; d̂]
Gn_w0 = abs(evalfr(tf([Tz 1], conv([Tp1 1], [Tp2 1])), 1i*w0));
fprintf('\n|Gn(j·w0)| = %.4f → senoide de W com amplitude %g °C dá ±%.2f °C em Tp em malha aberta\n', ...
        Gn_w0, AMP_W, AMP_W*Gn_w0);
fprintf('  (equivale a ±%.0f contagens de u; degrau AMP_DC = %g °C equivale a %.0f cont.)\n', ...
        AMP_W/Kp, AMP_DC, AMP_DC/Kp);
L12 = L(1:2);  L34 = L(3:4);  E = [1; 1];
for on = [1 0]
    Acl = [Ap,          Bp*Ki,  -Bp*K,                   -on*Bp*Cs;
           -Cp,         0,      zeros(1,2),              zeros(1,2);
           L12*Cp,      Bp*Ki,  Ap - Bp*K - L12*Cp,      zeros(2);
           L34*Cp,      zeros(2,1), -L34*Cp,             As];
    Bcl = [Bp*Nb  E  E;
           1      0  0;
           Bp*Nb  zeros(2,1) zeros(2,1);
           zeros(2,3)];
    Ccl = [Cp           0       zeros(1,2)   zeros(1,2);
           zeros(1,2)   Ki      -K           -on*Cs;
           zeros(1,5)                        Cs];
    Dcl = [0 0 0; Nb 0 0; 0 0 0];                % u = Nb·r + ... (passagem direta de r)
    sys = ss(Acl, Bcl, Ccl, Dcl);
    sys.InputName = {'r', 'Ta', 'W'};  sys.OutputName = {'y', 'u', 'dhat'};
    if on, sys_on = sys; else, sys_off = sys; end
    Hyw = evalfr(sys('y', 'W'), 1i*w0);
    fprintf('Observador %s: estável = %d, |Y/W(j·w0)| = %.2e → ±%.3f °C de Tp em regime\n', ...
            string(on), isstable(sys), abs(Hyw), abs(Hyw)*AMP_W);
end
Hdw = evalfr(sys_on('dhat', 'W'), 1i*w0);
fprintf('d̂/W em w0 (ligado): |·|·Kp = %.4f, fase = %.2f° (ideal: 1 e 0°, d̂ = W/Kp)\n', ...
        abs(Hdw)*Kp, angle(Hdw)*180/pi);
fprintf('Degrau de r (réplica): Tr = %.2f s, Mp = %.2f %%\n', ...
        stepinfo(sys_on('y', 'r')).RiseTime, stepinfo(sys_on('y', 'r')).Overshoot);

%% 7. Condições iniciais: equilíbrio no P.O. (r = PO, W = 0, Ta medida)
% Sem isso o .slx parte de Tp = 0 °C e o transitório inicial esconde o resto. O equilíbrio muda
% com o observador ligado/desligado (d̂ tem offset de Ta, que o observador não modela), por isso
% um vetor para cada caso; a seção 8 copia o certo para x1i, x2i, xii, x1ei, x2ei, xs1i, xs2i.
xeq = struct();
for on = [1 0]
    if on, sys = sys_on; else, sys = sys_off; end
    xe  = -sys.A \ (sys.B*[PO; Ta; 0]);
    ye  = sys.C*xe + sys.D*[PO; Ta; 0];
    xeq.(sprintf('on%d', on)) = xe;
    fprintf('Equilíbrio no P.O., observador %d: y = %.2f °C, u = %.0f cont., d̂ = %.0f cont.\n', on, ye);
end
fprintf('  (d̂ ≠ 0 em regime: offset de Ta, que o observador não modela; o integrador absorve o resto)\n');
x1i = 0; x2i = 0; xii = 0; x1ei = 0; x2ei = 0; xs1i = 0; xs2i = 0;   % a seção 8 sobrescreve

%% 8. Simulink: Prj2CDig.slx com observador ligado e desligado
% Cópia do modelo do professor nesta pasta. Alterações só em memória (o .slx não é salvo):
%   Step2 (degrau em 1000 s) = −AMP_DC, Step1 (senoide de 1800 a 3000 s) = −AMP_W,
%   "Manual Switch" (nome com quebra de linha no .slx) escolhe se d̂ é subtraído de u (sw = 0 ligado, 1 desligado).
% Referência: onda quadrada PO ± 5 °C, 0,00125 Hz (começa em PO − 5, troca a cada 400 s). Cada troca
% é um degrau de 10 °C (salto de Nb·10 ≈ 15000 em u): u sai de [0, umax], e o .slx não tem saturação.
mdl = 'Prj2CDig';
load_system(mdl);
set_param([mdl '/Step2'], 'After',  '-AMP_DC');
set_param([mdl '/Step1'], 'Before', '-AMP_W');
res = struct();
for on = OBS_ON
    set_param([mdl '/Manual ' newline 'Switch'], 'sw', num2str(1 - on));
    xe = xeq.(sprintf('on%d', on));
    x1i  = xe(1);  x2i  = xe(2);  xii  = xe(3);
    x1ei = xe(4);  x2ei = xe(5);  xs1i = xe(6);  xs2i = xe(7);
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on', 'MaxStep', '1');
    sd  = out.ScopeData;                         % entrada 1: [Us, e_sin]; entrada 2: [Ta, W, R, Y]
    t = sd.time;  U = sd.signals(1).values;  Y = sd.signals(2).values;
    r = struct('t', t, 'u', U(:,1), 'dhat', U(:,2), 'w', Y(:,2), 'r', Y(:,3), 'y', Y(:,4));
    % Tr e Mp no degrau da quadrada em 400 s (PO−5 → PO+5; a quadrada começa em −5)
    r0 = r.r(find(t < 400, 1, 'last'));  r1 = r.r(find(t > 400, 1));
    j = t >= 400 & t < 800;  tj = t(j) - 400;  yn = (r.y(j) - r0)/(r1 - r0);
    Tr_sim = tj(find(yn >= 0.9, 1)) - tj(find(yn >= 0.1, 1));
    Mp_sim = max(0, (max(yn) - 1))*100;
    % Senoide: janela de 2400 a 3000 s (a quadrada troca em 2400 e 2800: usa 2480–2790, após o transitório)
    js = t >= 2480 & t <= 2790;
    amp = (max(r.y(js) - r.r(js)) - min(r.y(js) - r.r(js)))/2;
    fprintf('\nSimulink, observador %s: Tr = %.1f s, Mp = %.1f %%, erro senoidal ±%.3f °C, u ∈ [%.0f, %.0f]\n', ...
            string(on), Tr_sim, Mp_sim, amp, min(r.u), max(r.u));
    if max(r.u) > umax || min(r.u) < umin
        fprintf('  aviso: u sai de [%d, %d] (o .slx não satura; anti-windup entra na Fase 2)\n', umin, umax);
    end
    res.(sprintf('on%d', on)) = r;

    figure('Name', sprintf('Prj2CDig contínuo, observador %s', string(on)));
    subplot(311); plot(t, r.r, 'k--', t, r.y); grid on; ylabel('T_p [°C]')
    legend('r', 'y', 'Location', 'best')
    title(sprintf('Simulink contínuo, observador %s (AMP\\_W = %g °C)', ...
          string(on), AMP_W))
    subplot(312); plot(t, r.u); yline(umax, 'r--'); grid on; ylabel('u [cont.]')
    subplot(313); plot(t, -r.w/Kp, t, r.dhat); grid on; ylabel('[cont.]'); xlabel('t [s]')
    legend('perturbação −W/Kp', 'd̂', 'Location', 'best')
    if SALVAR_FIG
        if ~isfolder('outputs/fase1'), mkdir('outputs/fase1'); end
        exportgraphics(gcf, sprintf('outputs/fase1/proj2cdig_continuo_obs%d.png', on), 'Resolution', 150);
    end
end
if all(isfield(res, {'on0', 'on1'}))
    figure('Name', 'Observador ligado × desligado');
    plot(res.on0.t, res.on0.y - res.on0.r, res.on1.t, res.on1.y - res.on1.r); grid on
    xline([1000 1800 3000], ':', {'degrau W', 'senoide', 'fim senoide'})
    ylabel('y − r [°C]'); xlabel('t [s]'); legend('observador desligado', 'observador ligado')
    title('Erro de seguimento: rejeição da perturbação senoidal')
    if SALVAR_FIG
        exportgraphics(gcf, 'outputs/fase1/proj2cdig_continuo_comparacao.png', 'Resolution', 150);
    end
end
% Para rodar à mão depois deste script: sim Prj2CDig (o modelo fica carregado com AMP_W/AMP_DC).
