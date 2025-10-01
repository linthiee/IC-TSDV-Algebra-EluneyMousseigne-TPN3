#include "raylib.h"

struct Figures
{
	Model model;

	Vector3 position = { 0, 0, 0 };
};

void main()
{

	Figures cube;
	Figures decahedron;
	Figures dodecahedron;
	Figures icosahedron;
	Figures octahedron;
	Figures tetrahedron;

	InitWindow(800, 600, "TP 3");

	Camera3D camera = { 0 };
	camera.position = { 4.0f, 4.0f, 4.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	float cameraSpeed = 0.2f;

	cube.model = LoadModel("res/cube.obj");
	decahedron.model = LoadModel("res/decahedron.obj");
	dodecahedron.model = LoadModel("res/dodecahedron.obj");
	icosahedron.model = LoadModel("res/icosahedron.obj");
	octahedron.model = LoadModel("res/octahedron.obj");
	tetrahedron.model = LoadModel("res/tetrahedron.obj");

	cube.position = { 0.0f, 0.0f, 0.0f, };
	decahedron.position = { 3.0f, 0.0f, 0.0f, };
	dodecahedron.position = { 6.0f, 0.0f, 0.0f, };
	icosahedron.position = { -7.0f, 0.0f, 0.0f, };
	octahedron.position = { -5.0f, 3.0f, 0.0f, };
	tetrahedron.position = { -3.0f, 5.0f, 0.0f, };

	SetTargetFPS(60);

	while (!WindowShouldClose())
	{
		UpdateCamera(&camera, CAMERA_FREE);
		DisableCursor();

		if (IsKeyDown(KEY_RIGHT))
		{
			camera.position.x += cameraSpeed;
		}
		if (IsKeyDown(KEY_LEFT))
		{
			camera.position.x -= cameraSpeed;
		}
		if (IsKeyDown(KEY_UP))
		{
			camera.position.z -= cameraSpeed;
		}
		if (IsKeyDown(KEY_DOWN))
		{
			camera.position.z += cameraSpeed;
		}
		if (IsKeyDown(KEY_Q))
		{
			camera.position.y += cameraSpeed;
		}
		if (IsKeyDown(KEY_E))
		{
			camera.position.y -= cameraSpeed;
		}


		BeginDrawing();
		ClearBackground(RAYWHITE);

		BeginMode3D(camera);

		DrawModel(cube.model, cube.position, 1.0f, RED);
		DrawModel(decahedron.model, decahedron.position, 1.0f, RED);
		DrawModel(dodecahedron.model, dodecahedron.position, 1.0f, RED);
		DrawModel(icosahedron.model, icosahedron.position, 1.0f, RED);
		DrawModel(octahedron.model, octahedron.position, 1.0f, RED);
		DrawModel(tetrahedron.model, tetrahedron.position, 1.0f, RED);

		DrawGrid(20, 1.0f);
		EndMode3D();

		EndDrawing();
	}


	UnloadModel(cube.model);
	UnloadModel(decahedron.model);
	UnloadModel(dodecahedron.model);
	UnloadModel(icosahedron.model);
	UnloadModel(octahedron.model);
	UnloadModel(tetrahedron.model);

	CloseWindow();

}
