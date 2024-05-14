#include <Arduino.h>
#include <stdio.h>



typedef struct vector {int x;int y;}vector2;
typedef struct pp {vector2* points;int size;} polygon;
int isInside(vector2 B, const polygon* p);

void setup() {
  Serial.begin(115200);

}

void loop() {

  // 다각형 예시1
    vector2 point1 = { 2356.2, 53.884 };
    vector2 point2 = { 2430.5, 46.997 };
    vector2 point3 = { 2584.9, 41.259 };
    vector2 point4 = { 2768.2, 37.244 };
    vector2 point5 = { 2995.2, 33.516 };
    vector2 point6 = { 3207.4, 31.223 };
    vector2 point7 = { 3441.3, 30.509 };
    vector2 point8 = { 3718.9, 30.226 };
    vector2 point9 = { 4025.6, 30.804 };
    vector2 point10 = { 4310.3, 31.526 };
    vector2 point11 = { 4514.5, 32.964 };
    vector2 point12 = { 4821.1, 34.116 };
    vector2 point13 = { 5047.2, 35.841 };
    vector2 point14 = { 5273.3, 37.567 };
    vector2 point15 = { 5492, 40.44 };
    vector2 point16 = { 5637.6, 43.025 };
    vector2 point17 = { 5673.5, 46.613 };
    vector2 point18 = { 5644, 49.224 };
    vector2 point19 = { 5453.4, 51.536 };
    vector2 point20 = { 5233.5, 53.417 };
    vector2 point21 = { 4977, 54.146 };
    vector2 point22 = { 4698.4, 54.73 };
    vector2 point23 = { 4383.2, 55.316 };
    vector2 point24 = { 4031.4, 56.047 };
    vector2 point25 = { 3708.8, 56.778 };
    vector2 point26 = { 3349.7, 57.076 };
    vector2 point27 = { 3063.8, 57.373 };
    vector2 point28 = { 2711.9, 8 };
    vector2 point29 = { 2356.2, 8 };

    vector2 points1[] = { point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11, point12, point13, point14, point15, point16, point17, point18, point19, point20, point21, point22, point23, point24, point25, point26, point27, point28, point29 };
    polygon poly1 = { points1, 29 };



   // 다각형 예시2
    vector2 point30 = { 8000, 79.25 };
    vector2 point31 = { 5338.7, 79.779 };
    vector2 point32 = { 3135.6, 80.146 };
    vector2 point33 = { 2131.5, 73.698 };
    vector2 point34 = { 1690.2, 64.06 };
    vector2 point35 = { 1540.6, 49.656 };
    vector2 point36 = { 1500.5, 33.375 };
    vector2 point37 = { 1652.9, 27.75 };
    vector2 point38 = { 2749.8, 23.244 };
    vector2 point39 = { 4638.1, 22.599 };
    vector2 point40 = { 6410.5, 27.723 };
    vector2 point41 = { 7623, 49.725 };
    vector2 point42 = { 8000, 59.941 };

    vector2 points2[] = { point30, point31, point32, point33, point34, point35, point36, point37, point38, point39, point40, point41 , point42 };
    polygon poly2 = { points2, 11 };

    // 첫 번째 다각형 내부에 있는지 확인
    vector2 testPoint1 = { 4000, 50 };
    int isInsidePolygon1 = isInside(testPoint1, &poly1);
    // 두 번째 다각형 내부에 있는 점 확인
    vector2 testPoint2 = { 4000, 50 };
    int isInsidePolygon2 = isInside(testPoint2, &poly2);

    if (isInsidePolygon1 && isInsidePolygon2){
        Serial.println("96%");
    }
    else{
        Serial.println("94% \n");
    }
}




int isInside(vector2 B, const polygon* p) {
    int crosses = 0;
    for (int i = 0; i < p->size; i++) {
        int j = (i + 1) % p->size;
        if ((p->points[i].y > B.y) != (p->points[j].y > B.y)) {
            double atX = (p->points[j].x - p->points[i].x) * (B.y - p->points[i].y) / (p->points[j].y - p->points[i].y) + p->points[i].x;
            if (B.x < atX)
                crosses++;
        }
    }
    //Serial.println(crosses);
    return crosses % 2 > 0;
}

    