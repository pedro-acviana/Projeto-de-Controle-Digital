function R = proj1cdig_auto_projeto(arqCSV, grupo, Sat, arqJSON)
% proj1cdig_auto_projeto — identificação + projeto do PI discreto, sem interface.
%
% Chamada pelo proj1cdig_auto.py (matlab -batch) ou à mão na Command Window:
%   proj1cdig_auto_projeto('dados/proj1cdig_auto_prbs_....csv', 2, 16000, 'dados/projeto.json')
%
% Mesma lógica das seções 1–7 do proj1cdig_m.m (pasta proj1cdig):
%   procest P2DZ com varredura de partições → polo dominante de Mp/Tp → PI discreto
%   (Ts = Td/2, condição de ângulo em z + refino) → simulação com saturação ±AW.
% Grava o resultado em JSON e as figuras em dados/figs_projeto/.

if nargin < 4, arqJSON = 'dados/projeto.json'; end
figdir = fullfile(fileparts(arqJSON), 'figs_projeto');
if ~exist(figdir, 'dir'), mkdir(figdir); end
avisos = {};

% ---- grupo ----
g = mod(grupo-1, 10) + 1;
tabW0 = [0.065 0.065 0.065 0.065 0.065 0.098 0.098 0.098 0.098 0.098];
tabPO = [45 50 55 60 45 50 55 60 45 50];
tabMp = [10 10 10 25 25 30 30 30 15 15];
tabTp = [25 25 25 30 25 25 30 30 25 30];
PO = tabPO(g); Mp_esp = tabMp(g); Tp_esp = tabTp(g); w0 = tabW0(g); %#ok<NASGU>

% ---- dados ----
T = readtable(arqCSV, 'TextType', 'string');
T = T(T.modo == "PRBS", :);
transit = 2000;                                % tempo de aquecimento, descartado
T = T(T.k >= transit, :);
um = T.u; ym = T.tp; Ta = mean(T.ta, 'omitnan');
ok = ~isnan(ym) & ~isnan(um);
um = um(ok); ym = ym(ok);
uid = um - mean(um); yid = ym - mean(ym);
Ti = 1;

f = figure('Visible', 'off');
subplot(211); plot((0:numel(ym)-1)/3600, ym); ylabel('Tp [°C]'); grid on; title('Coleta PRBS (após aquecimento)')
subplot(212); plot((0:numel(um)-1)/3600, um); ylabel('u [cont.]'); xlabel('t [h]'); grid on
saveas(f, fullfile(figdir, 'coleta_prbs.png')); close(f)

% ---- identificação ----
ini = idproc('P2DZ');
ini.Structure.Td.Minimum = 0;  ini.Structure.Td.Maximum = 20;
ini.Structure.Tz.Minimum = 0;
melhor.fit = -Inf;
for i = 1:19
    len_id = ceil((100 - 5*i)/100 * numel(uid));
    data  = iddata(yid(1:len_id),   uid(1:len_id),   Ti);
    datav = iddata(yid(len_id:end), uid(len_id:end), Ti);
    if numel(datav.y) < 600, continue, end
    ge = procest(data, ini);
    [~, fit] = compare(datav, ge);
    fprintf('partição %2d%%: fit %6.2f %%  Td %5.2f s\n', 5*i, fit, ge.Td);
    if fit > melhor.fit, melhor.fit = fit; melhor.ge = ge; melhor.i = i; melhor.datav = datav; end
end
ge = melhor.ge;
f = figure('Visible', 'off'); compare(melhor.datav, ge);
title(sprintf('Validação: %d%% dos dados, fit %.2f %%', 5*melhor.i, melhor.fit))
saveas(f, fullfile(figdir, 'validacao.png')); close(f)

Kp = ge.Kp; Tz = ge.Tz; Tp1 = ge.Tp1; Tp2 = ge.Tp2; Td = ge.Td;
if Tp2 > Tp1, [Tp1, Tp2] = deal(Tp2, Tp1); end
if Td < 4 || Td > 8, avisos{end+1} = sprintf('Td = %.2f s fora de 4–8 s', Td); end
if Tz <= 0, avisos{end+1} = 'Tz <= 0'; end
G = tf(Kp*[Tz 1], conv([Tp1 1], [Tp2 1]), 'InputDelay', Td);

% ---- especificação → polo dominante ----
Mp = Mp_esp/100;
zeta = -log(Mp)/sqrt(pi^2 + log(Mp)^2);
wn   = pi/(Tp_esp*sqrt(1 - zeta^2));
sd   = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);

% ---- PI contínuo (ponto de partida, só informativo) ----
Gs  = polyval(Kp*[Tz 1], sd) / polyval(conv([Tp1 1],[Tp2 1]), sd) * exp(-Td*sd);
phi = mod(-pi - angle(Gs) + angle(sd), 2*pi);
a   = -real(sd) + imag(sd)/tan(phi);
KP  = 1/abs((sd + a)/sd * Gs);  KI = KP*a;

% ---- PI discreto ----
Ts  = round(Td/2, 2);
Gz  = absorbDelay(c2d(G, Ts, 'zoh'));
zd  = exp(sd*Ts);
Gzd = evalfr(Gz, zd);
phi = mod(-pi - angle(Gzd) + angle(zd - 1), 2*pi);
z0a = real(zd) - imag(zd)/tan(phi);
Ka  = 1/abs((zd - z0a)/(zd - 1) * Gzd);
best = [Inf z0a Ka];
for z0i = linspace(max(0.5, z0a - 0.15), 0.999, 80)
    for Ki = Ka*linspace(0.2, 2, 60)
        Tcl = feedback(tf(Ki*[1 -z0i], [1 -1], Ts)*Gz, 1);
        if ~isstable(Tcl), continue, end
        S = stepinfo(Tcl);
        c = ((S.Overshoot - Mp_esp)/Mp_esp)^2 + ((S.PeakTime - Tp_esp)/Tp_esp)^2;
        if c < best(1), best = [c z0i Ki]; end
    end
end
z0 = best(2); K = best(3);
Cz = tf(K*[1 -z0], [1 -1], Ts);
S  = stepinfo(feedback(Cz*Gz, 1));

f = figure('Visible', 'off'); rlocus(Cz*Gz); zgrid(zeta, wn*Ts); hold on
plot(real(zd), [imag(zd) -imag(zd)], 'rx', 'MarkerSize', 12, 'LineWidth', 2);
title(sprintf('LGR discreto: K = %.2f, z0 = %.4f, Ts = %.2f s', K, z0, Ts))
saveas(f, fullfile(figdir, 'lgr_discreto.png')); close(f)

% ---- simulação com saturação ±AW ----
U0 = (PO - Ta)/Kp;
[A,B,C,~] = ssdata(ss(Gz));
N = round(900/Ts); t = (0:N-1)'*Ts; pico = [0 0];
f = figure('Visible', 'off');
for j = 1:2
    aw = (j == 1);
    x = (eye(size(A)) - A) \ (B*U0); I = U0; y = zeros(N,1); u = y;
    for k = 1:N
        y(k) = C*x + Ta;  r = PO + 5*(t(k) >= 30);
        e = r - y(k); uk = K*e + I; hi = uk > Sat; lo = uk < 0;
        if ~(aw && ((hi && e > 0) || (lo && e < 0))), I = I + K*(1 - z0)*e; end
        u(k) = min(max(uk, 0), Sat);  x = A*x + B*u(k);
    end
    pico(j) = max(y);
    subplot(211); hold on; stairs(t, y); subplot(212); hold on; stairs(t, u);
end
subplot(211); yline(PO + 5, 'k--'); legend('com AW', 'sem AW'); ylabel('Tp [°C]'); grid on
title(sprintf('Degrau de 5 °C com saturação em %d', Sat))
subplot(212); yline(Sat, 'r--'); ylabel('u [cont.]'); xlabel('t [s]'); grid on
saveas(f, fullfile(figdir, 'antiwindup.png')); close(f)
if U0 > Sat, avisos{end+1} = 'u de regime estimado acima de Sat'; end

% ---- saída ----
R = struct('Kp', Kp, 'Tz', Tz, 'Tp1', Tp1, 'Tp2', Tp2, 'Td', Td, 'fit', melhor.fit, ...
    'Ta', Ta, 'zeta', zeta, 'wn', wn, 'KP_cont', KP, 'KI_cont', KI, ...
    'Ts', Ts, 'K', K, 'z0', z0, 'z0_angulo', z0a, 'K_angulo', Ka, ...
    'Mp_sim', S.Overshoot, 'tp_sim', S.PeakTime, 'pico_aw1', pico(1), 'pico_aw0', pico(2), ...
    'U0_estimado', U0, 'avisos', {avisos});
fid = fopen(arqJSON, 'w'); fprintf(fid, '%s', jsonencode(R)); fclose(fid);
fprintf('\nResultado gravado em %s\n', arqJSON);
end
