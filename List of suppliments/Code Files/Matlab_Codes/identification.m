clear all
close all 
delete(instrfindall);

%identifikace parametru
% nastav PWM - cekej na ustaleni teploty
filePrefix = 'TEMPchamber_identif_';
%% pøíprava názvu souboru pro SAVE
files = dir;        %najdi soubor s nejvyssim poradovym cislem
nextFile = 0;
for i = 1:size(files,1)
    if strfind(files(i).name,filePrefix) > 0   %hledej jen soubory zacinajici na tento text
        ib = strfind(files(i).name,'.mat');      %najdi index pripony
        nextFile_ = sscanf(files(i).name(ib-2:ib-1),'%d');   %precti hodnotu posledni casti
        if nextFile_ > nextFile, nextFile = nextFile_; end  %vyhledej jen nejvetsi
    end
end
clear i ib nextFile_
fileName = [filePrefix num2str(nextFile+1,'%03d')]; %nový název s èíslem +1


%% tato sekvence smaze vsechny otevrene com porty
comstring = 'COM9';

if ~exist('ser','var')

    a = instrfindall;
    for i = 1:size(a,2)
        if contains(a(i).type, 'serial')
            if contains(a(i).port, comstring) %smaze jen com8
                delete(a(i))
            end
        end
    end


    %% open serial port
    ser = serial(comstring);
    set(ser,'baudrate',9600,'databits',8,'parity','none','stopbits',1,'terminator','CR/LF','timeout',2)
end
if ~strcmpi(get(ser,'Status'),'open')
    fopen(ser);
end

bytesAvailable = ser.BytesAvailable; %flush buffer
if bytesAvailable
    fread(ser,bytesAvailable);
    disp(['buffet flushed ' num2str(bytesAvailable) ' bytes'])
end


%% measuring loop
FanPWM = 255;
fwrite(ser,['FAN:PWM ' num2str(FanPWM) char(10)])
disp(['---------------- FAN:PWM ' num2str(FanPWM)])



res = [];
figure(1)

testMode = 1; %1 .. normal regulation, 0.. direct PWM
StepDurationMin = 5;  %in minutes .. duration of one step

if testMode == 1
    %tsetVals = [12:40:12:40];
    tsetVals = [55 40 20 30 40 50 60 70];
    endTimes = [1:length(tsetVals)] * StepDurationMin;
    noSteps = length(tsetVals);
    clear PWMvals
    
else
    PWMvals = linspace(-98,98,10); %hodnoty PWM na kterych se ma testovat
    % PWMvals = fliplr(PWMvals);
    endTimes = [1:length(PWMvals)] * StepDurationMin;
    noSteps = length(PWMvals);
    clear tsetVals
end

disp(['MEASUREMENT DURATION ' num2str(endTimes(end)/60) ' hours'])
measStartTime = now;
aOld = [0; 0; 0; 0; 0;0];
for k = 1:noSteps  %prostupne testuj ruzne hodnoty PWM
    lastBlockTimeStart = now;
    if testMode
        fwrite(ser,['REG:TEMP ' num2str(tsetVals(k)) char(10)])
        disp(['---------------- REG:TSET ' num2str(tsetVals(k))])
    else
        fwrite(ser,['REG:PWM ' num2str(PWMvals(k)) char(10)])
        disp(['---------------- REG:PWM ' num2str(PWMvals(k))])
    end

    while 1  %v ramci jedne hodnoty mer porad dokola
        bytesAvailable = ser.BytesAvailable;
        if bytesAvailable > 70
            chData = fread(ser,bytesAvailable);
            chData = char(chData');
            while chData(1) ~= 'n' %first character must be 'n'
                chData = chData(2:end); %withdraw one character from buffer
                if isempty(chData), break; end
            end
            if length(chData) > 0

                % a = sscanf(chData,'%f,%f,%f,%f, PWM %d, FANh %d, FANc %d, mode %d');
                % n:41.60 t1:18.12 t2:24.00 t3:17.62 t4:29.50 tset:15.00 Preg:1.5625 Ireg:0.56000
                a = sscanf(chData,'n:%f t1:%f t2:%f t3:%f t4:%f tset:%f Preg:%f Ireg:%f');

                if length(a) < 8
                    disp(['Short message: ' num2str(length(a)) ';  bytes: ' num2str(bytesAvailable)])
                else
                    %a(a(2:5)== -127) = aOld(a(2:5)== -127); %fix probem with One wire sensors
                    aOld = a;
                    res(end+1,:) = [now, a', FanPWM];
                    

                    figure(1)
                    ts = (res(:,1) - res(1,1))*24*60;
                    res_ = res;
                    res_(res_ == -127) = NaN;
                    %plot(ts, res_(:,2),'.-', ts, res_(:,7),'.-',...
                      %  ts, res_(:,3), ts, res_(:,4), ts, res_(:,5), ts, res_(:,6))

                    plot(ts, res_(:,2),'.-', ts, res_(:,7),'.-',...
                        ts, res_(:,3),ts, res_(:,8),ts, res_(:,9))
                    grid on
                    zoom on
                    xlabel('t (min)')
                    ylabel('temp (°C, PWM')
                    %legend('PWM','tSet','T1 inner air','T2 ambient','T3 inner heatsink','T4 cold heatsink','Location','northwest')
                    legend('PWM','tSet','T1 inner air','Perror','Ierror','Location','northwest')
                    disp([num2str(ts(end),'%5.2f') ': ' num2str(res(end,2)) ', ' num2str(res(end,3)) ', ' num2str(res(end,4)) ', ' num2str(res(end,5)) ', ' num2str(res(end,6)) ', tset ' num2str(res(end,7)) ', Preg ' num2str(res(end,8)) ', Ireg ' num2str(res(end,9))])
                    
                    figure(2)
                    plot(ts, res_(:,7),ts, res_(:,3))
                    grid on
                    zoom on
                    xlabel('t (min)')
                    ylabel('temp (°C)')
                    legend('tSet','T1 inner air','Location','northwest')



                    tempLimit = 85;
                    if res(end,3) > tempLimit
                        save(fileName,'res')
                        disp(['TEMP LIMIT   --------- File SAVED:  ' fileName])
                        break
                    end

                    if  (now - res(1,1))*24*60 > endTimes(k) %prekroceni casu = preskoc na dalsi hodnotu PWM
                        save(fileName,'res')
                        disp(['File SAVED:  ' fileName])
                        break
                    end

                end
            else
                if size(res,2) > 1
                    if (now - res(end,1))*24*60*60 > 2
                        disp('No data for 2 seconds')
                    end
                end
            end


            %     setTemp = 15.34;
            %     fwrite(ser,['REG:TEMP ' num2str(setTemp) char(10)])
            %
            %     t(end+1) = now;
            %
            %     a = getReg(ser,10);
            %     Temp(end+1) = sscanf(a(strfind(a,'=')+1:end),'%f');
            %     a = getReg(ser,12);
            %     pU(end+1) = sscanf(a(strfind(a,'=')+1:end),'%f');
            %     a = getReg(ser,13);
            %     pI(end+1) = sscanf(a(strfind(a,'=')+1:end),'%f');
            %
            %     ts = (t - t(1))*24*60;
            %     plot(ts, Temp, ts, pU, ts, pI*10)
            %     grid on
            %     zoom on
            %     xlabel('t (min)')
            %     ylabel('temp, U, I')
            %     legend('temp','Peltier U','Peltier I/10')
            %     ax = axis;
            %     text((ax(2)-ax(1))*0.05 + ax(1), (ax(4)-ax(3))*0.9 + ax(3), ['temp = ' num2str(Temp(end),4) ' °C'])
            %     text((ax(2)-ax(1))*0.05 + ax(1), (ax(4)-ax(3))*0.8 + ax(3), ['U = ' num2str(pU(end),4) ' V'])
            %     if pI > 0, heatTxt = 'COOL'; else heatTxt = 'HEAT'; end
            %     text((ax(2)-ax(1))*0.05 + ax(1), (ax(4)-ax(3))*0.7 + ax(3), ['I = ' num2str(pI(end),4) ' A   ' heatTxt])
            %
        end
        pause(.2)
    end

    if res(end,3) > tempLimit
        break
    end


end


fwrite(ser,['REG:PWM ' num2str(0) char(10)])
disp(['---------------- REG:PWM ' num2str(0)])

fwrite(ser,['REG:TEMP ' num2str(20) char(10)])
disp(['---------------- REG:PWM ' num2str(0)])

fwrite(ser,['FAN:PWM ' num2str(0) char(10)])
disp(['---------------- FAN:PWM ' num2str(0)])


fclose(ser)