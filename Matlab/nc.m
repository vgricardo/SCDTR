endereco = 'COM10'; %Endereço da porta série
NSAMPLES = 200;

Ts = 0.002992; %Período de amostragem (para desenhar a escala de tempo nos gráficos)

if exist('sp1') %Fecha descritores da porta série anteriores
    fclose(sp1);
    delete(sp1);
    clear sp1;
end

%Inicializa a porta série
sp1 = serial(endereco);
set(sp1, 'BaudRate', 115200);
set(sp1,'Terminator','LF'); % define o terminador para o println

fopen(sp1); %Abre a porta série
pause(3); %Espera que o Arduino inicie
flushinput(sp1); %Limpa o buffer da porta série

fwrite(sp1, '050'); %Pede nova referência. Nota: a referência deve ter 3 carateres: 50 <=> '050'
y = [];
u = [];
for i=1:(2*NSAMPLES)
    linha = fgetl(sp1);
    linha(linha == 13) = [];
    linha(linha == 10) = [];
    if mod(i, 2) == 0 %O programa envia alternadamente u e y
        u = [u, hex2dec(linha)];
    else
        y = [y, hex2lx(linha)];
    end
    
end

%Nova referência
% flushinput(sp1)
% fwrite(sp1, '050');
% for i=1:(2*NSAMPLES)
%    linha = fgetl(sp1);
%    linha(linha == 13) = [];
%    linha(linha == 10) = [];
%    if mod(i, 2) == 0
%        u = [u, hex2dec(linha)];
%    else
%        y = [y, hex2lx(linha)];
%    end
% end


ref = 100*ones(1, length(y));
ref2 = [200*ones(1, length(y)/2), 50*ones(1,length(y)/2)]; %Alterar também o valor da referência aqui! (para mostrar bem no gráfico)
plot(0:Ts:size(y, 2)/(1/Ts)-Ts, y);
hold on;
plot(0:Ts:size(y, 2)/(1/Ts)-Ts, ref, 'r');
legend('Saída do sistema', 'Referência')
xlabel('Tempo [s]');
ylabel('Saída [lux]');
figure(2);
plot(0:Ts:size(u, 2)/(1/Ts)-Ts, u);
xlabel('Tempo [s]');
ylabel('Atuação [%]');
ylim([0, 110]);
fclose(sp1);
delete(sp1);
clear sp1;