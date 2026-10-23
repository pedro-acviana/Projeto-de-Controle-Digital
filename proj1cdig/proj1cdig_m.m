% proj1cdig_m.m — Prj1CDig: identificação + projeto do PI anti-windup discreto
% ENE0167 Controle Digital · UnB
%
% Baseado no Prj1CDig_m.m do Prof. Adolfo (Referencias/Prj1CDig), adaptado para:
%   * ler o CSV do proj1cdig_logger.py em vez do ma31500.mat;
%   * calcular o ponto de partida do PI pelo LGR (condição de ângulo e de módulo),
%     contínuo e discreto, antes de abrir o sisotool;
%   * simular o PI discreto com saturação COM e SEM anti-windup (mesma lei do firmware);
%   * imprimir as linhas prontas para colar em firmware/main/proj1cdig_config.h;
%   * medir Mp e tp no ensaio real em malha fechada.
%
% Rode seção por seção (Ctrl+Enter). Copie Prj1CDig.slx e Termico_FP.slx da pasta
% Referencias para esta pasta (não edite os originais).
%
% Unidade do atuador: contagens do PWM de 16 bits, 0..65535 (= 0..12 V médio), como pede o
% roteiro atualizado. Kp sai em °C/contagem (~0,003) e o ganho do PI em contagens/°C (~2000).

clear; close all; clc

%% 0. Parâmetros do grupo  <<< TROQUE AQUI >>>
GRUPO = 2;                    % grupo 2: PO 50 °C, Mp 10 %, Tp 25 s, w0 0,065
ARQ_PRBS = 'dados/proj1cdig_prbs_2026-10-20_23-26-20.csv';
ARQ_MF = 'dados/proj1cdig_ensaio_aw0_2026-10-21_11-56-47.csv';               % CSV do ensaio em malha fechada (seção 9), '' para pular
Sat      = 21000;             % = U_MAX do firmware (3 × 150 Ω a 12 V: 32 % de 65535, 0,31 W por resistor)
ABRIR_SISOTOOL = false;       % true para abrir o Control System Designer nas seções 5 e 6
U0_MEDIDO = 10200;            % u que deixou Tp estável no P.O. (Etapa F, 20/10)

g      = mod(GRUPO-1, 10) + 1;
tabW0  = [0.065 0.065 0.065 0.065 0.065 0.098 0.098 0.098 0.098 0.098];
tabPO  = [45 50 55 60 45 50 55 60 45 50];
tabMp  = [10 10 10 25 25 30 30 30 15 15];
tabTp  = [25 25 25 30 25 25 30 30 25 30];
w0 = tabW0(g); PO = tabPO(g); Mp_esp = tabMp(g); Tp_esp = tabTp(g);
fprintf('Grupo %d: PO = %g °C, Mp = %g %%, Tp = %g s, w0 = %g rad/s\n', GRUPO, PO, Mp_esp, Tp_esp, w0);

%% 1. Carregar a coleta PRBS
Ti = 1;                                         % período de amostragem da identificação
T  = readtable(ARQ_PRBS, 'TextType', 'string');
T  = T(T.modo == "PRBS", :);
um = T.u;  ym = T.tp;  tam = T.ta;
tm = (0:numel(um)-1)' * Ti;
Ta = mean(tam, 'omitnan');                      % temperatura ambiente média do ensaio
fprintf('%d amostras (%.2f h). Ta média = %.2f °C (min %.2f, max %.2f)\n', ...
        numel(um), numel(um)/3600, Ta, min(tam), max(tam));

figure('Name','Coleta PRBS');
subplot(311); plot(tm/3600, ym); ylabel('Tp [°C]'); grid on; title('Coleta PRBS')
subplot(312); plot(tm/3600, tam); ylabel('Ta [°C]'); grid on
subplot(313); plot(tm/3600, um); ylabel('u [cont.]'); xlabel('t [h]'); grid on

%% 2. Identificação P2DZ (procest)
transit = 2000;                                 % descarta o aquecimento até o P.O. (= warm_s)
USAR_TP_MENOS_TA = true;                        % identifica Tp − Ta: tira a deriva da sala
                                                % (ex.: queda de ~0,9 °C em Ta ao amanhecer, na coleta de 20–21/10)
ui = um(transit:end);
if USAR_TP_MENOS_TA, yi = ym(transit:end) - tam(transit:end); else, yi = ym(transit:end); end
uid = ui - mean(ui);  yid = yi - mean(yi);

% Chute inicial e limites na faixa do roteiro. Sem eles o procest cai num mínimo local
% (Tz = 0, Tp1 ~ 40 s, Td ~ 2 s, fit menor) que perde a cauda lenta da resposta.
ini = idproc('P2DZ');
ini.Structure.Kp.Value  = 0.002;  ini.Structure.Kp.Minimum  = 0;
ini.Structure.Tp1.Value = 300;    ini.Structure.Tp1.Minimum = 50;  ini.Structure.Tp1.Maximum = 2000;
ini.Structure.Tp2.Value = 40;     ini.Structure.Tp2.Minimum = 5;   ini.Structure.Tp2.Maximum = 200;
ini.Structure.Tz.Value  = 200;    ini.Structure.Tz.Minimum  = 0;   ini.Structure.Tz.Maximum  = 2000;
ini.Structure.Td.Value  = 4;      ini.Structure.Td.Minimum  = 0;   ini.Structure.Td.Maximum  = 15;

melhor = struct('fit', -Inf);
for i = 1:19                                    % partições identificação/validação
    len_id = ceil((100 - 5*i)/100 * numel(uid));
    data  = iddata(yid(1:len_id),   uid(1:len_id),   Ti);
    datav = iddata(yid(len_id:end), uid(len_id:end), Ti);
    if numel(datav.y) < 600, continue, end       % validação curta demais
    ge = procest(data, ini);
    [~, fit] = compare(datav, ge);
    fprintf('  partição %2d%% validação: fit = %6.2f %%  Td = %5.2f s\n', 5*i, fit, ge.Td);
    if fit > melhor.fit, melhor = struct('fit', fit, 'ge', ge, 'i', i, 'datav', datav); end
end
ge = melhor.ge;
figure('Name','Validação'); compare(melhor.datav, ge);
title(sprintf('Melhor partição: %d%% validação, fit %.2f %%', 5*melhor.i, melhor.fit))

Kp = ge.Kp; Tz = ge.Tz; Tp1 = ge.Tp1; Tp2 = ge.Tp2; Td = ge.Td;
if Tp2 > Tp1, [Tp1, Tp2] = deal(Tp2, Tp1); end   % Tp1 = polo lento (ambiente)
fprintf('\nModelo: Kp=%.5f °C/cont  Tz=%.2f s  Tp1=%.2f s  Tp2=%.2f s  Td=%.3f s\n', Kp, Tz, Tp1, Tp2, Td);
if Td < 3 || Td > 9
    warning('Td = %.2f s fora da faixa esperada pelo roteiro (4–8 s). Confira ruído e fixação.', Td);
end

G = tf(Kp*[Tz 1], conv([Tp1 1], [Tp2 1]), 'InputDelay', Td);

%% 3. Variáveis usadas por Termico_FP.slx e Prj1CDig.slx
Ts = Ti;                       % Termico_FP usa "Ts" = 1 s (a seção 6 redefine Ts!)
W = 0;
Am = (Tz - Tp1)/(Tp2 - Tp1);  Bm = 1 - Am;      % forma modal
Ct = 0.0002; Ka = 1; Kw = 4; Kt = 1/(Ct*Tp1) - Ka - Kw;   % (Ka+Kt+Kw)*Ct = 1/Tp1
Kp2 = 1/(Kt*Ct*Tp1);                                      %#ok<NASGU>
if w0 < 0.08, AmpW0 = 0.02; else, AmpW0 = 0.02*0.981/0.654; end   % valores do roteiro
% Modelo caixa-cinza contra os dados medidos (figura útil no vídeo):
% open Termico_FP; sim Termico_FP
U0 = (PO - Ta)/Kp;                              % u de regime no P.O. (estimativa)
fprintf('u estimado no P.O.: %.0f contagens (%.1f %% de duty). Sat = %d.\n', U0, U0/655.35, Sat);

%% 4. Especificações → polo dominante desejado
Mp = Mp_esp/100;
zeta = -log(Mp)/sqrt(pi^2 + log(Mp)^2);
wn   = pi/(Tp_esp*sqrt(1 - zeta^2));
sd   = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);
fprintf('\nζ = %.4f  ωn = %.4f rad/s  polo desejado s = %.4f ± j%.4f\n', zeta, wn, real(sd), imag(sd));

%% 5. PI contínuo pelo LGR:  C(s) = KP (s + a)/s = KP + KI/s
Gs  = polyval(Kp*[Tz 1], sd) / polyval(conv([Tp1 1],[Tp2 1]), sd) * exp(-Td*sd);
phi = mod(-pi - angle(Gs) + angle(sd), 2*pi);   % ângulo que o zero do PI precisa dar
a   = -real(sd) + imag(sd)/tan(phi);
KP  = 1/abs((sd + a)/sd * Gs);
KI  = KP*a;
fprintf('PI contínuo (LGR): zero em s = %.5f, KP = %.3f cont/°C, KI = %.4f cont/(°C·s)\n', -a, KP, KI);
if a <= 0, warning('Zero do PI no semiplano direito: relaxe Tp ou Mp no sisotool.'); end
Cs = tf(KP*[1 a], [1 0]);
Tcl = feedback(Cs*G, 1);  S = stepinfo(Tcl);
fprintf('  → simulação linear: Mp = %.1f %%, tp = %.1f s (o zero e os outros polos mudam a resposta: ajuste no sisotool)\n', S.Overshoot, S.PeakTime);
if ABRIR_SISOTOOL, controlSystemDesigner('rlocus', G, Cs); end
% Depois de ajustar no sisotool, exporte C e atualize KP e KI antes de rodar o Prj1CDig.slx.
% Confira no diagrama se o bloco Ki está em paralelo (C = KP + KI/s), como assumido aqui.

%% 6. PI discreto pelo LGR em z:  C(z) = K (z - z0)/(z - 1),  Ts = Td/2
Ts  = round(Td/2, 3);                            % Ts = Td/2 (passo 6a do roteiro)
Gd  = G;  Gd.InputDelay = 2*Ts;                  % atraso = exatamente 2 amostras: evita o zero
Gz  = absorbDelay(c2d(Gd, Ts, 'zoh'));           % espúrio (z ~ -300) do atraso fracionário
zd  = exp(sd*Ts);
Gzd = evalfr(Gz, zd);
phi = mod(-pi - angle(Gzd) + angle(zd - 1), 2*pi);
z0  = real(zd) - imag(zd)/tan(phi);
K   = 1/abs((zd - z0)/(zd - 1) * Gzd);
fprintf('\nTs = %.2f s. Polo desejado em z = %.4f ± j%.4f\n', Ts, real(zd), imag(zd));
fprintf('PI discreto (condição de ângulo): z0 = %.5f, K = %.3f\n', z0, K);
Cz = tf(K*[1 -z0], [1 -1], Ts);
S  = stepinfo(feedback(Cz*Gz, 1));
fprintf('  → Mp = %.1f %%, tp = %.1f s\n', S.Overshoot, S.PeakTime);

% Refino: varre z0 e K perto do ponto de partida, procurando Mp e tp da especificação.
best = [Inf z0 K];
for z0i = linspace(max(0.5, z0 - 0.15), 0.999, 80)
    for Ki = K*linspace(0.2, 2, 60)
        Tz_ = feedback(tf(Ki*[1 -z0i], [1 -1], Ts)*Gz, 1);
        if ~isstable(Tz_), continue, end
        Si = stepinfo(Tz_);
        c = ((Si.Overshoot - Mp_esp)/Mp_esp)^2 + ((Si.PeakTime - Tp_esp)/Tp_esp)^2;
        if c < best(1), best = [c z0i Ki]; end
    end
end
z0 = best(2); K = best(3);
Cz = tf(K*[1 -z0], [1 -1], Ts);
S  = stepinfo(feedback(Cz*Gz, 1));
fprintf('PI discreto refinado: z0 = %.5f, K = %.3f → Mp = %.1f %%, tp = %.1f s\n', z0, K, S.Overshoot, S.PeakTime);
figure('Name','LGR discreto'); rlocus(Cz*Gz); zgrid(zeta, wn*Ts); hold on
hd = plot(real(zd)*[1 1], [imag(zd) -imag(zd)], 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'LineStyle', 'none');
title('LGR de C(z)G(z)')
pmf = pole(feedback(Cz*Gz, 1));                 % polos de malha fechada com o K escolhido
hm = plot(real(pmf), imag(pmf), 'ms', 'MarkerSize', 10, 'LineWidth', 1.5, 'LineStyle', 'none');
legend([hd hm], {'polo desejado (Mp, tp)', 'polos de MF (K, z0 finais)'}, 'Location', 'northwest')
axis equal; xlim([-1.1 1.1]); ylim([-1.1 1.1])
fprintf('Polos de MF: %s\n', mat2str(pmf.', 4));
if ABRIR_SISOTOOL, controlSystemDesigner('rlocus', Gz, Cz); end
% Use o sisotool para justificar no vídeo a posição do zero e do ganho (passos 6b e 6c).

%% 7. Simulação discreta com saturação: com e sem anti-windup
% Modelo linear em torno do ponto de operação MEDIDO (u0 = U0_MEDIDO, Tp = PO):
%   Tp = PO + y_desvio,  y_desvio = Gz * (u - U0_MEDIDO),  u saturado em [0, Sat]
% (o ganho local Kp é menor que (PO - Ta)/u0, então (PO - Ta)/Kp superestimaria u0)
U0 = U0_MEDIDO;
Ts = Gz.Ts;                                      % garante o Ts do PI discreto (a seção 3 faz Ts = 1)
[A,B,C,D] = ssdata(ss(Gz));
N  = round(900/Ts);  t = (0:N-1)'*Ts;
fprintf('\nu necessário em regime: %.0f para PO+5, %.0f para PO+10 (Sat = %d)\n', U0 + 5/Kp, U0 + 10/Kp, Sat);
for dr = [5 10]                                  % degrau de 5 °C e o da Figura 5 (PO → PO+10)
    figure('Name', sprintf('Anti-windup, degrau de %d °C', dr)); 
    for aw = [1 0]
        x = zeros(size(A,1), 1);  I = U0;  y = zeros(N,1); u = y;
        for k = 1:N
            y(k) = C*x + PO;
            r = PO + dr*(t(k) >= 30);
            e = r - y(k);  uk = K*e + I;  hi = uk > Sat;  lo = uk < 0;
            if ~(aw && ((hi && e > 0) || (lo && e < 0))), I = I + K*(1 - z0)*e; end
            u(k) = min(max(uk, 0), Sat);
            x = A*x + B*(u(k) - U0);
        end
        subplot(211); hold on; stairs(t, y); subplot(212); hold on; stairs(t, u);
        fprintf('degrau %2d °C, AW=%d: pico %.2f °C (sobrepasso %5.1f %%), saturado %4.0f s, final %.2f °C\n', ...
                dr, aw, max(y), (max(y) - PO - dr)/dr*100, sum(u >= Sat - 1)*Ts, y(end));
    end
    subplot(211); yline(PO + dr, 'k--'); ylabel('Tp [°C]'); legend('com AW', 'sem AW', 'Location', 'southeast'); grid on
    title(sprintf('Degrau de %d °C a partir do P.O. (u0 = %d), saturação em %d', dr, U0, Sat))
    subplot(212); yline(Sat, 'r--'); ylabel('u [cont.]'); xlabel('t [s]'); grid on
end
% Para o passo 7 do roteiro, monte também a versão discreta no Simulink: copie
% Prj1CDig.slx, troque o PI contínuo por um Discrete Transfer Fcn / blocos com Ts,
% mantendo a saturação 0-Sat e o chaveamento do anti-windup
% (set_param('Prj1CDig/Anti-WindUp','sw','1') = sem AW, '0' = com AW).

%% 8. Linhas para colar em firmware/main/proj1cdig_config.h
fprintf('\n/* ---- gerado por proj1cdig_m.m em %s ---- */\n', datestr(now));
fprintf('#define GROUP_PO_C         %.1ff\n', PO);
fprintf('#define GROUP_W0_RAD_S     %.3ff\n', w0);
fprintf('#define CTRL_TS_MS         %d\n', round(Ts*1000));
fprintf('#define PI_K               %.4ff\n', K);
fprintf('#define PI_Z0              %.5ff\n', z0);
fprintf('/* ou, sem regravar: k %.4f %.5f */\n', K, z0);

%% 9. Ensaio em malha fechada: Mp, tp e ISE medidos
% Para o ensaio do roteiro (comando "e" do firmware, Figura 5): o arquivo começa no instante
% do comando, então t = 0 é o início do ensaio e o ISE é calculado entre 2100 e 3000 s.
if ~isempty(ARQ_MF)
    M = readtable(ARQ_MF, 'TextType', 'string');
    M = M(M.modo == "CTRL", :);
    i0 = find(M.k == 0, 1);  if isempty(i0), i0 = 1; end   % início do ensaio (k zera no comando)
    M = M(i0:end, :);
    t = M.t_s - M.t_s(1);
    dt = [diff(t); median(diff(t))];
    if t(end) >= 3000
        jise = t >= 2100 & t < 3000;
        ISE = sum((M.ref(jise) - M.tp(jise)).^2 .* dt(jise));
        fprintf('ISE (2100–3000 s) = %.1f °C²·s   (roteiro: típico 2000–4000 no Simulink)\n', ISE);
    end
    fprintf('Tempo com o aquecedor saturado: %.0f s\n', sum(dt(M.u >= Sat - 1)));
    kd = find(abs(diff(M.ref)) > 0.5, 1);        % primeiro degrau de referência
    if isempty(kd)
        warning('Nenhum degrau de referência no arquivo.');
    else
        r0 = M.ref(kd); r1 = M.ref(kd+1); t0 = t(kd+1);
        kprox = find(abs(diff(M.ref(kd+1:end))) > 0.5, 1);   % próximo degrau (onda quadrada)
        tfim = t0 + 600; if ~isempty(kprox), tfim = min(tfim, t(kd + kprox)); end
        jan = t >= t0 & t <= tfim;
        [yp, ip] = max(sign(r1 - r0) * M.tp(jan)); yp = sign(r1 - r0)*yp;
        tj = t(jan);
        fprintf('Ensaio: Mp = %.1f %% (esp. %g %%), tp = %.1f s (esp. %g s)\n', ...
            (yp - r1)/(r1 - r0)*100, Mp_esp, tj(ip) - t0, Tp_esp);
    end
    figure('Name','Malha fechada');
    subplot(311); plot(t, M.tp, t, M.ref, 'k--'); ylabel('Tp [°C]'); grid on; legend('Tp','ref')
    subplot(312); stairs(t, M.u); yline(Sat, 'r--'); ylabel('u [cont.]'); grid on
    subplot(313); plot(t, M.w); ylabel('cooler [cont.]'); xlabel('t [s]'); grid on
end
