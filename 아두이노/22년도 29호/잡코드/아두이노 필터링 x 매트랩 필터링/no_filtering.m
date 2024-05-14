clear all
close all 
clc

aa=fopen('DATALOG.TXT','r');
data = fscanf(aa,'%f', [4,10000]);data = data';
time = data(:,1); n = length(time);
sensor1 = data(:,2); sensor2 = data(:,3); sensor3 = data(:,4);
a=input('Enter the number: ');  % 값이 높아질 수록 정확 그러나 딜레이 생김
t2=ones(1,a);num=(1/a)*t2;den=[1];
filtered_sensor1=filter(num,den,sensor1);
filtered_sensor2=filter(num,den,sensor2);
filtered_sensor3=filter(num,den,sensor3);
plot(time, filtered_sensor1, time, filtered_sensor2, time, filtered_sensor3);
%%xlim([(a+1)*(time(2)-time(1)) time(n)+3]);   %이동평균 필터 사용시 초반부분 필터링 필요
xlabel('Time (T)');ylabel("Temperature (C)");
legend('sensor1','sensor2','sensor3'); grid on
fclose(aa);


