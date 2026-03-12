#ifndef PHYSICS_H
#define PHYSICS_H

#include "raylib.h"

typedef struct FlyingObject {
    Vector3 pos; // meters
    Vector3 velocity; // m/s
    Vector3 external_force; // N
    float mass; // kg
    float thrust; // N
    float altitude; // meters
    float aoa; // Angle of Attack (degrees)
    float reference_area; // meters

    float aoa_to_cdrag[40]; // angle of attack to drag coefficient from -10 to 29 degrees
    float aoa_to_clift[40]; // angle of attack to lift coefficient from -10 to 29 degrees
} FlyingObject;


void fobj_update(FlyingObject *fobj, float dt, Vector3 forward_dir);
void fobj_set_test_aero_tables(FlyingObject *fobj);
void fobj_init(FlyingObject *fobj);

Vector3 vector_3_normalize(Vector3 v);
Vector3 vector_3_negate(Vector3 v);
Vector3 vector_3_crossproduct(Vector3 v1, Vector3 v2);
float vector_3_length(Vector3 v);
float vector3_length_squared(Vector3 v);
Vector3 vector3_scale_safe(Vector3 v, float s);
Vector3 vector3_add_safe(Vector3 a, Vector3 b);
Vector3 vector3_sub_safe(Vector3 a, Vector3 b);
float deg_to_rad(float deg);


#endif