#include "test_simulations.h"
#include "raylib.h"
//#include "physics.c"
#include "physics.h"
#include "math.h"


void draw_simple_plane(Vector3 pos, Vector3 forward) {
    DrawCapsuleWires(pos, vector3_add_safe(forward, pos), 1.0f, 10, 10, RED);
}


void generate_landscape(Vector3 pos, int num_blocks, float block_size) {
    float start_x =  ceilf(pos.x / block_size) * block_size;
    float start_y =  ceilf(pos.z / block_size) * block_size;
    float end_x = start_x + (block_size * num_blocks);
    float end_y = start_y + (block_size * num_blocks);
    for (float current_x = start_x  - (num_blocks * block_size); current_x < end_x; current_x += block_size) {
        for (float current_y = start_y - (num_blocks * block_size); current_y < end_y; current_y += block_size) {
            DrawCube((Vector3){current_x,25-block_size,current_y}, block_size, block_size, block_size, GREEN);
            DrawCube((Vector3){current_x,5,current_y}, 1, 10, 1, BROWN);
            DrawCube((Vector3){current_x,10,current_y}, 5, 5, 5, DARKGREEN);
        }
    }
    
}


void test_simple_flight() {
    InitWindow(1000, 800, "Flight Physics Test");
    SetTargetFPS(60);

    FlyingObject plane;
    fobj_init(&plane);
	

    plane.mass = 10.0f;
    plane.thrust = 1.0f;
    plane.reference_area = 0.05f;
    plane.pos = (Vector3){ 0.0f, 10.0f, 0.0f };
    plane.velocity = (Vector3){ 1.0f, 1.0f, 1.0f };
    plane.aoa = 0.0f;
	//plane.altitude = 100.0f;
    //plane.external_force = (Vector3) {0.0f, 80.0f, 0.0f};

    Vector3 forward = { 40.0f, 0.0f, 0.0f };
    forward = vector_3_normalize(forward);
    //forward = vector3_add_safe(forward, (Vector3){1.0f, 0.0f, 0.0f});

    Camera3D cam = { 0 };
    cam.position = (Vector3){ 40.0f, 40.0f, 40.0f };
    cam.target = (Vector3){ 0.0f, 5.0f, 0.0f };
    cam.up = (Vector3){ 0.0f, 5.0f, 0.0f };
    cam.fovy = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        //plane.thrust_input = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;

        float up_force = 0;
        float right_force = 0;

        if (IsKeyDown(KEY_UP)) up_force = 10000*dt;
        if (IsKeyDown(KEY_DOWN)) up_force = 10000*dt;
        if (IsKeyDown(KEY_RIGHT)) right_force = 2000*dt;
        if (IsKeyDown(KEY_LEFT)) right_force = -2000*dt;
		
        if (IsKeyDown(KEY_SPACE)) plane.thrust += 1 * dt;
        if (IsKeyDown(KEY_X)) plane.thrust -= 1 * dt;


        fobj_update(&plane, dt, forward);
		//plane.thrust = 0.01f;

        //plane.external_force = vector3_add_safe(plane.external_force, (Vector3) {0,up_force,0});
        // z -z y
        plane.external_force = (Vector3) {forward.z*right_force, up_force, forward.x*right_force};
        //plane.external_force = (Vector3) {-forward.x*up_force, -forward.z*up_force, forward.y*up_force};
        //plane.external_force = (Vector3) {0.0f, 0.5f * vector_3_length(plane.velocity), 0.0f};

        up_force = 0.0f;
        right_force = 0.0f;
		forward = vector_3_normalize(plane.velocity);
        //forward = vector3_add_safe(forward, (Vector3){5.0f, 0.0f, 0.0f});
        //forward = vector_3_normalize(forward);
		cam.target = plane.pos;
		cam.position = (Vector3) {plane.pos.x - (forward.x*100) ,
             plane.pos.y - (forward.y*100) >= 0 ? plane.pos.y - (forward.y*100) : 0,
              plane.pos.z - (forward.z*100) };//{plane.pos.x - forward.x, plane.pos.y - forward.y, plane.pos.z - forward.z};

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(cam);
        //DrawGrid(10000, 1.0f);
        //DrawSphere(plane.pos, 0.3f, RED);
        draw_simple_plane(plane.pos, forward);
        //DrawLine3D((Vector3){0,0,0}, plane.pos, YELLOW);
        DrawLine3D(plane.pos, vector3_add_safe(plane.pos, plane.external_force), PURPLE);

        //DrawCube((Vector3){0,0,30}, 10.0f, 10.0f, 10.0f, BROWN);
        generate_landscape(plane.pos, 10, 50.0f);
        EndMode3D();


        DrawText(TextFormat("Altitude: %.2f m", plane.altitude), 20, 20, 20, BLACK);
        DrawText(TextFormat("Speed: %.2f m/s", vector_3_length(plane.velocity)), 20, 50, 20, BLACK);
        DrawText(TextFormat("AOA: %.2f deg", plane.aoa), 20, 80, 20, BLACK);
        DrawText(TextFormat("pos: x: %.2f y: %.2f", plane.pos.x, plane.pos.z), 20, 100, 20, BLACK);
        DrawText("SPACE = thrust, UP/DOWN = change AOA", 20, 130, 20, DARKGRAY);

        
        EndDrawing();
    }

    CloseWindow();
}

