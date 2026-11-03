
#include <math.h>
#include "ExtraMath.hpp"

//finds the magnitude of an array of size 3
float mag(float array[]) {
    float mag = 0;
    for (int i = 0; i < 3; i++) {
        mag = mag + pow(array[i], 2);  //adds component i of array squared
    }
    mag = sqrt(mag);
    return mag;
}

//calculates the dot product of two arrays
float dot(float array1[], float array2[]) {
    return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}