#include "physics.h"
#include "math.h"

#define GRAVITY 9.81f
#define SEA_LEVEL_AIR_DENSITY 1.225f


static float clampf(float x, float min_val, float max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}


/* Vector Math */

Vector3 vector_3_normalize(Vector3 v) {
    float magnitude = sqrt(v.x*v.x + v.y*v.y + v.z*v.z );

    //return (Vector3) {-1*v.x, -1*v.y, -1*v.z};
    return (Vector3) {v.x / magnitude, v.y / magnitude, -v.z / magnitude};
}

Vector3 vector_3_negate(Vector3 v) {
    return (Vector3) {-1*v.x, -1*v.y, -1*v.z};
}

Vector3 vector_3_crossproduct(Vector3 v1, Vector3 v2) {
    /*
    a2xb3 - a3xb2
    a3xb1 - a1xb3
    a1xb2 - a2xb1
    */
    return (Vector3) {
        v1.y*v2.z - v1.z*v2.y,
        v1.z*v2.x - v1.x-v2.z,
        v1.x*v2.y - v1.y*v2.x
    };
} 

float vector_3_length(Vector3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z );
}


static float vector3_length_squared(Vector3 v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static Vector3 vector3_scale_safe(Vector3 v, float s) {
    Vector3 out = { v.x * s, v.y * s, v.z * s };
    return out;
}

static Vector3 vector3_add_safe(Vector3 a, Vector3 b) {
    Vector3 out = { a.x + b.x, a.y + b.y, a.z + b.z };
    return out;
}

static Vector3 vector3_sub_safe(Vector3 a, Vector3 b) {
    Vector3 out = { a.x - b.x, a.y - b.y, a.z - b.z };
    return out;
}

static float deg_to_rad(float deg) {
    return deg * (3.14159265358979323846f / 180.0f);
}

/* fobj functions */

void fobj_init(FlyingObject *fobj) {
    if (!fobj) return;

    fobj->pos = (Vector3){ 0.0f, 0.0f, 0.0f };
    fobj->velocity = (Vector3){ 0.0f, 0.0f, 0.0f };

    fobj->mass = 1.0f;
    fobj->thrust = 0.0f;
    fobj->reference_area = 0.01f;

    fobj->altitude = 0.0f;
    fobj->aoa = 0.0f;

    //fobj_set_default_aero_tables(fobj);
}


void fobj_set_test_aero_tables(FlyingObject *fobj) {
    if (!fobj) return;

    for (int i = 0; i < 40; i++) {
        float aoa = (float)(i - 10);  // -10 to +29

        float cl = 0.1f * aoa;
        if (aoa > 15.0f) {
            cl = 1.5f - 0.05f * (aoa - 15.0f); // crude stall behavior
        }
        if (aoa < -5.0f) {
            cl = 0.1f * aoa;
        }

        float cd = 0.02f + 0.0025f * aoa * aoa;

        fobj->aoa_to_clift[i] = cl;
        fobj->aoa_to_cdrag[i] = cd;
    }
}

float air_density_from_altitude(float altitude) {
    // Simple exponential atmosphere model
    // rho = rho0 * exp(-h/H), H approx 8500 m
    const float scale_height = 8500.0f;
    if (altitude < 0.0f) altitude = 0.0f;
    return SEA_LEVEL_AIR_DENSITY * expf(-altitude / scale_height);
}

float fobj_get_drag_coefficient(const FlyingObject *fobj, float aoa_deg) {
    if (!fobj) return 0.0f;

    int index = (int)floorf(aoa_deg) + 10;
    index = (int)clampf((float)index, 0.0f, 39.0f);
    return fobj->aoa_to_cdrag[index];
}

float fobj_get_lift_coefficient(const FlyingObject *fobj, float aoa_deg) {
    if (!fobj) return 0.0f;

    int index = (int)floorf(aoa_deg) + 10;
    index = (int)clampf((float)index, 0.0f, 39.0f);
    return fobj->aoa_to_clift[index];
}

Vector3 fobj_compute_gravity_force(const FlyingObject *fobj) {
    if (!fobj) return (Vector3){ 0.0f, 0.0f, 0.0f };

    return (Vector3){ 0.0f, -fobj->mass * GRAVITY, 0.0f };
}


Vector3 fobj_compute_thrust_force(const FlyingObject *fobj, Vector3 forward_dir) {
    if (!fobj) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float len_sq = vector3_length_squared(forward_dir);
    if (len_sq < 1e-8f) return (Vector3){ 0.0f, 0.0f, 0.0f };

    forward_dir = vector_3_normalize(forward_dir);

    return vector3_scale_safe(forward_dir, fobj->thrust);
}

Vector3 fobj_compute_drag_force(const FlyingObject *fobj) {
    if (!fobj) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float speed = vector_3_length(fobj->velocity);
    if (speed < 1e-5f) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float rho = air_density_from_altitude(fobj->altitude);
    float cd = fobj_get_drag_coefficient(fobj, fobj->aoa);

    float drag_magnitude = 0.5f * rho * speed * speed * cd * fobj->reference_area;

    Vector3 drag_dir = vector_3_negate(vector_3_normalize(fobj->velocity));
    return vector3_scale_safe(drag_dir, drag_magnitude);
}

Vector3 fobj_compute_lift_force(const FlyingObject *fobj, Vector3 lift_dir) {
    if (!fobj) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float speed = vector_3_length(fobj->velocity);
    if (speed < 1e-5f) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float len_sq = vector3_length_squared(lift_dir);
    if (len_sq < 1e-8f) return (Vector3){ 0.0f, 0.0f, 0.0f };

    float rho = air_density_from_altitude(fobj->altitude);
    float cl = fobj_get_lift_coefficient(fobj, fobj->aoa);

    float lift_magnitude = 0.5f * rho * speed * speed * cl * fobj->reference_area;

    lift_dir = vector_3_normalize(lift_dir);
    return vector3_scale_safe(lift_dir, lift_magnitude);
}

void fobj_update(FlyingObject *fobj, float dt, Vector3 forward_dir) {
    if (!fobj || dt <= 0.0f) return;
    if (fobj->mass <= 0.0f) return;

    // Update altitude from current position
    // Assuming Y is the vertical axis
    fobj->altitude = fobj->pos.y;
    if (fobj->altitude < 0.0f) fobj->altitude = 0.0f;

    // Normalize forward direction
    if (vector3_length_squared(forward_dir) < 1e-8f) {
        forward_dir = (Vector3){ 1.0f, 0.0f, 0.0f };
    } else {
        forward_dir = vector_3_normalize(forward_dir);
    }

    // Use world up to define lift direction
    Vector3 world_up = { 0.0f, 1.0f, 0.0f };

    // Build a lift direction perpendicular to velocity and sideways axis
    Vector3 velocity_dir;
    if (vector_3_length(fobj->velocity) < 1e-5f) {
        velocity_dir = forward_dir;
    } else {
        velocity_dir = vector_3_normalize(fobj->velocity);
    }

    Vector3 side = vector_3_crossproduct(velocity_dir, world_up);
    if (vector3_length_squared(side) < 1e-8f) {
        side = (Vector3){ 0.0f, 0.0f, 1.0f };
    } else {
        side = vector_3_normalize(side);
    }

    Vector3 lift_dir = vector_3_crossproduct(side, velocity_dir);
    if (vector3_length_squared(lift_dir) < 1e-8f) {
        lift_dir = world_up;
    } else {
        lift_dir = vector_3_normalize(lift_dir);
    }

    Vector3 gravity_force = fobj_compute_gravity_force(fobj);
    Vector3 thrust_force  = fobj_compute_thrust_force(fobj, forward_dir);
    Vector3 drag_force    = fobj_compute_drag_force(fobj);
    Vector3 lift_force    = fobj_compute_lift_force(fobj, lift_dir);

    Vector3 net_force = { 0.0f, 0.0f, 0.0f };
    net_force = vector3_add_safe(net_force, gravity_force);
    net_force = vector3_add_safe(net_force, thrust_force);
    net_force = vector3_add_safe(net_force, drag_force);
    net_force = vector3_add_safe(net_force, lift_force);

    Vector3 acceleration = vector3_scale_safe(net_force, 1.0f / fobj->mass);

    // Semi-implicit Euler integration
    fobj->velocity = vector3_add_safe(fobj->velocity, vector3_scale_safe(acceleration, dt));
    fobj->pos = vector3_add_safe(fobj->pos, vector3_scale_safe(fobj->velocity, dt));

    // Prevent going below ground
    if (fobj->pos.y < 0.0f) {
        fobj->pos.y = 0.0f;
        if (fobj->velocity.y < 0.0f) {
            fobj->velocity.y = 0.0f;
        }
    }

    fobj->altitude = fobj->pos.y;
}
