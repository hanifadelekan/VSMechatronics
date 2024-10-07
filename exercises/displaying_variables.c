//including libraries
#include <stdio.h>
#include <assert.h>
#include <math.h>

int main(void)

{
    //declaring variables
    int a = 1, b = 2;
    float c = 1.23f;
    char* check =  "correct";

    

    //return 0 to show program ran successfully

    //calculating volume

    float pi = M_PI;

    float r = 10;

    float volume = 4 * pi * r*r*r / 3;

    float s_area = 4 * pi * r*r;

    //printing the variables
    printf("The variables are\na = %d\nb = %d and\nc = %f\nCheck: %s\n The volume is %.2f and the surface area is %.2f",a,b,c,check,volume,s_area);


    return 0 ;

}