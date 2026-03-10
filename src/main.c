#include "raylib.h"

#include "resource_dir.h"// utility header for SearchAndSetResourceDir

#include "physics.c"
#include "physics.h"

int main ()
{
	InitWindow(1000, 800, "Flight Physics Test");
    SetTargetFPS(60);

    FlyingObject rocket;
    fobj_init(&rocket);
	

    rocket.mass = 10.0f;
    rocket.thrust = 200.0f;
    rocket.reference_area = 0.05f;
    rocket.pos = (Vector3){ 0.0f, 10.0f, 0.0f };
    rocket.velocity = (Vector3){ 0.0f, 40.0f, 1.0f };
    rocket.aoa = 0.0f;
	//rocket.altitude = 100.0f;

    Vector3 forward = { 40.0f, 0.0f, 0.0f };
    forward = vector_3_normalize(forward);

    Camera3D cam = { 0 };
    cam.position = (Vector3){ 40.0f, 40.0f, 40.0f };
    cam.target = (Vector3){ 0.0f, 5.0f, 0.0f };
    cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    cam.fovy = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        //rocket.thrust_input = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;

        if (IsKeyDown(KEY_UP)) rocket.aoa += 20.0f * dt;
        if (IsKeyDown(KEY_DOWN)) rocket.aoa -= 20.0f * dt;
		if (IsKeyDown(KEY_SPACE)) rocket.thrust += 20.0f * dt;

        fobj_update(&rocket, dt, forward);
		rocket.thrust = 0.01f;

		forward = vector_3_normalize(rocket.velocity);
		cam.target = rocket.pos;
		cam.position = (Vector3) {rocket.pos.x - (forward.x*20), rocket.pos.y - (forward.y*20), rocket.pos.z - (forward.z*20)};//{rocket.pos.x - forward.x, rocket.pos.y - forward.y, rocket.pos.z - forward.z};

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(cam);
        DrawGrid(100, 10.0f);
        DrawSphere(rocket.pos, 0.3f, RED);
        EndMode3D();

        DrawText(TextFormat("Altitude: %.2f m", rocket.altitude), 20, 20, 20, BLACK);
        DrawText(TextFormat("Speed: %.2f m/s", vector_3_length(rocket.velocity)), 20, 50, 20, BLACK);
        DrawText(TextFormat("AOA: %.2f deg", rocket.aoa), 20, 80, 20, BLACK);
        DrawText("SPACE = thrust, UP/DOWN = change AOA", 20, 110, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
