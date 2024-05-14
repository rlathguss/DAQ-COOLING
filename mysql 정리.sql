select * from example1.sensor
where Motor_Temp >= 50;
-- select 모든 칼럼을 example1.sensor 테이블에서 where 모터 온도가 50도 이상인 것을 불러 와라

select * from example1.sensor
order by Date desc;
-- select 모든 칼럼을 example1.sensor 테이블에서 order by Date 기준으로 내림차순 정렬

select * from example1.sensor 
where RPM >= 2000 limit 5;
-- 모든칼럼에서 rpm의 값이 2000이상인인 상위 5개 데이터만 출력하라

select * from example1.sensor
where  RPM >= 2000 order by Date desc;
-- 모든칼럼에서 rpm의 값이 2000이상인인 데이터를 날짜 기준 내림차순으로 출력하라;

SELECT MAX(DCcurrent) FROM example1.sensor;

SELECT MIN(DCcurrent) FROM example1.sensor;


delete from sensor order by Date asc limit 500;