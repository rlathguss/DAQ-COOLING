-- SHOW VARIABLES LIKE "secure_file_priv" 해당 쿼리 먼저 실행 해서 저장가능 위치 파악
-- select "Date" UNION 칼럼명을 cvs 파일에 출력하기 위한 쿼리
select "Date", "RPM", "Motor_Temp", "PowerStage_Temp", "LPM","Torque_Out","I_des","ACpower1","ACcurrent","DCvoltage","ACvoltage","ACpower2","DCpower","DCcurrent","Xaccle", "Yaccle","Xangle","Yangle","Rad","CoolantTemp1","CoolantTemp2","Status" UNION select * from sensor INTO OUTFILE 'C:\\ProgramData\\MySQL\\MySQL Server 8.0\\Uploads\\logdata.csv' FIELDS ENCLOSED BY '"' TERMINATED BY',' ESCAPED BY '"' LINES TERMINATED BY '\n';

