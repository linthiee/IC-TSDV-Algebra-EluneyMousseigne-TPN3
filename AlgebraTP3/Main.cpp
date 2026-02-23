#include "math.h"
#include <vector>
#include <cmath>
#include <float.h>
#include <string>
#include <iostream>
#include "raymath.h"

const int gridDivisions = 3;

struct Figure
{
	std::string name;
	Model model;
	MyAABB localAABB;

	Vector3 position;
	Vector3 scale;
	Vector3 rotAxis;
	float rotAngle;

	Matrix worldMatrix;
	MyAABB worldAABB;

	std::vector<Plane> allNormals;
};

void DrawAABB(MyAABB aabb, Color color);
void Draw(const int figureCount, Figure allFigures[6], Color modelColors[6], bool finalCollision, bool broadPhaseCollision, Figure* controlledFigure, std::vector<Vector3>& gridPoints);

void cameraControl(Camera3D& camera, float cameraSpeed);

void figureSelector(Figure*& controlledFigure, Figure allFigures[6]);

void figureManipulation(Figure* controlledFigure);

void main()
{
	const int figureCount = 6;
	Figure allFigures[figureCount];

	InitWindow(800, 600, "TP 3 Algebra");

	Camera3D camera = { 0 };
	camera.position = { 4.0f, 4.0f, 4.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	float cameraSpeed = 0.2f;

	allFigures[0].name = "Cube";
	allFigures[0].model = LoadModel("res/cube.obj");

	allFigures[1].name = "Decahedron";
	allFigures[1].model = LoadModel("res/decahedron.obj");

	allFigures[2].name = "Dodecahedron";
	allFigures[2].model = LoadModel("res/dodecahedron.obj");

	allFigures[3].name = "Icosahedron";
	allFigures[3].model = LoadModel("res/icosahedron.obj");

	allFigures[4].name = "Octahedron";
	allFigures[4].model = LoadModel("res/octahedron.obj");

	allFigures[5].name = "Tetrahedron";
	allFigures[5].model = LoadModel("res/tetrahedron.obj");

	for (int i = 0; i < figureCount; i++)
	{
		if (allFigures[i].model.meshCount == 0)
		{
			CloseWindow();

			std::cout << "Error: Failed to load model " << allFigures[i].name.c_str() << "\n";
		}

		CalculateUniqueModelNormal(allFigures[i].model, allFigures[i].allNormals);

		allFigures[i].localAABB = CalculateLocalAABB(allFigures[i].model.meshes[0]);
		allFigures[i].scale = { 1.0f, 1.0f, 1.0f };
		allFigures[i].rotAxis = { 0.0f, 1.0f, 0.0f };
		allFigures[i].rotAngle = 0.0f;
	}

	allFigures[0].position = { 0.0f, 0.0f, 0.0f, };
	allFigures[1].position = { 3.0f, 0.0f, 0.0f, };
	allFigures[2].position = { 6.0f, 0.0f, 0.0f, };
	allFigures[3].position = { -7.0f, 0.0f, 0.0f, };
	allFigures[4].position = { -5.0f, 3.0f, 0.0f, };
	allFigures[5].position = { -3.0f, 5.0f, 0.0f, };

	Figure* controlledFigure = &allFigures[0];
	Color modelColors[] = { RED, BLUE, GREEN, MAGENTA, YELLOW, SKYBLUE };

	bool finalCollision = false;
	bool broadPhaseCollision = false;
	std::vector<Vector3> gridPoints;

	SetTargetFPS(60);

	while (!WindowShouldClose())
	{
		UpdateCamera(&camera, CAMERA_FREE);
		DisableCursor();

		cameraControl(camera, cameraSpeed);
		figureSelector(controlledFigure, allFigures);

		if (controlledFigure)
		{
			figureManipulation(controlledFigure);
		}

		finalCollision = false;
		broadPhaseCollision = false;
		gridPoints.clear();

		if (!controlledFigure)
		{
			continue;
		}

		Matrix matScaleA = MatrixScale(controlledFigure->scale.x, controlledFigure->scale.y, controlledFigure->scale.z);
		Matrix matRotA = MatrixRotate(controlledFigure->rotAxis, controlledFigure->rotAngle * DEG2RAD);
		Matrix matTransA = MatrixTranslate(controlledFigure->position.x, controlledFigure->position.y, controlledFigure->position.z);

		controlledFigure->worldMatrix = MatrixMultiply(MatrixMultiply(matScaleA, matRotA), matTransA);
		controlledFigure->worldAABB = GetUpdatedAABB(controlledFigure->localAABB, controlledFigure->worldMatrix);

		for (int i = 0; i < figureCount; i++)
		{
			Figure* other = &allFigures[i];
			if (other == controlledFigure) continue;

			Matrix matScaleB = MatrixScale(other->scale.x, other->scale.y, other->scale.z);
			Matrix matRotB = MatrixRotate(other->rotAxis, other->rotAngle * DEG2RAD);
			Matrix matTransB = MatrixTranslate(other->position.x, other->position.y, other->position.z);

			other->worldMatrix = MatrixMultiply(MatrixMultiply(matScaleB, matRotB), matTransB);
			other->worldAABB = GetUpdatedAABB(other->localAABB, other->worldMatrix);

			bool aabbCollision = CheckCollisionAABB(controlledFigure->worldAABB, other->worldAABB);

			if (aabbCollision)
			{
				broadPhaseCollision = true;

				MyAABB intersectionBB;
				intersectionBB.min.x = fmaxf(controlledFigure->worldAABB.min.x, other->worldAABB.min.x);
				intersectionBB.min.y = fmaxf(controlledFigure->worldAABB.min.y, other->worldAABB.min.y);
				intersectionBB.min.z = fmaxf(controlledFigure->worldAABB.min.z, other->worldAABB.min.z);
				intersectionBB.max.x = fminf(controlledFigure->worldAABB.max.x, other->worldAABB.max.x);
				intersectionBB.max.y = fminf(controlledFigure->worldAABB.max.y, other->worldAABB.max.y);
				intersectionBB.max.z = fminf(controlledFigure->worldAABB.max.z, other->worldAABB.max.z);

				Vector3 size = Vector3Subtract(intersectionBB.max, intersectionBB.min);

				Vector3 step;
				if (gridDivisions < 2)
				{
					step = { 0.0f, 0.0f, 0.0f };
				}
				else
				{
					step.x = size.x / (gridDivisions - 1);
					step.y = size.y / (gridDivisions - 1);
					step.z = size.z / (gridDivisions - 1);
				}

				bool stopChecking = false;

				for (int iz = 0; iz < gridDivisions; iz++)
				{
					if (stopChecking)
					{
						break;
					}

					float z = (gridDivisions < 2) ? intersectionBB.min.z + size.z * 0.5f : intersectionBB.min.z + step.z * iz;

					for (int iy = 0; iy < gridDivisions; iy++)
					{
						if (stopChecking)
						{
							break;
						}
						float y = (gridDivisions < 2) ? intersectionBB.min.y + size.y * 0.5f : intersectionBB.min.y + step.y * iy;

						for (int ix = 0; ix < gridDivisions; ix++)
						{
							float x = (gridDivisions < 2) ? intersectionBB.min.x + size.x * 0.5f : intersectionBB.min.x + step.x * ix;

							Vector3 point = { x, y, z };
							gridPoints.push_back(point);

							bool inA = IsPointInsideMesh(point, controlledFigure->model, controlledFigure->worldMatrix, controlledFigure->allNormals);
							bool inB = IsPointInsideMesh(point, other->model, other->worldMatrix, other->allNormals);

							if (inA && inB)
							{
								finalCollision = true;
								stopChecking = true;
								break;
							}

							if (gridDivisions < 2)
							{
								break;
							}
						}
						if (gridDivisions < 2)
						{
							break;
						}
					}
					if (gridDivisions < 2)
					{
						break;
					}
				}
			}

			if (finalCollision)
			{
				break;
			}
		}

		BeginDrawing();
		ClearBackground(RAYWHITE);

		BeginMode3D(camera);

		Draw(figureCount, allFigures, modelColors, finalCollision, broadPhaseCollision, controlledFigure, gridPoints);

		EndDrawing();
	}

	for (int i = 0; i < figureCount; i++)
	{
		UnloadModel(allFigures[i].model);
	}

	CloseWindow();
}

void figureManipulation(Figure* controlledFigure)
{
	if (IsKeyDown(KEY_J)) controlledFigure->position.x -= 0.1f;
	if (IsKeyDown(KEY_L)) controlledFigure->position.x += 0.1f;
	if (IsKeyDown(KEY_I)) controlledFigure->position.z -= 0.1f;
	if (IsKeyDown(KEY_K)) controlledFigure->position.z += 0.1f;
	if (IsKeyDown(KEY_R)) controlledFigure->position.y += 0.1f;
	if (IsKeyDown(KEY_F)) controlledFigure->position.y -= 0.1f;
	if (IsKeyDown(KEY_U)) controlledFigure->rotAngle -= 1.0f;
	if (IsKeyDown(KEY_O)) controlledFigure->rotAngle += 1.0f;
	if (IsKeyDown(KEY_Y))
	{
		controlledFigure->scale = Vector3Add(controlledFigure->scale, { 0.05f, 0.05f, 0.05f });
	}
	if (IsKeyDown(KEY_H))
	{
		controlledFigure->scale = Vector3Subtract(controlledFigure->scale, { 0.05f, 0.05f, 0.05f });

		const float minScale = 0.05f;
		if (controlledFigure->scale.x < minScale) controlledFigure->scale.x = minScale;
		if (controlledFigure->scale.y < minScale) controlledFigure->scale.y = minScale;
		if (controlledFigure->scale.z < minScale) controlledFigure->scale.z = minScale;
	}
}

void figureSelector(Figure*& controlledFigure, Figure  allFigures[6])
{
	if (IsKeyDown(KEY_ONE)) controlledFigure = &allFigures[0];
	if (IsKeyDown(KEY_TWO)) controlledFigure = &allFigures[1];
	if (IsKeyDown(KEY_THREE)) controlledFigure = &allFigures[2];
	if (IsKeyDown(KEY_FOUR)) controlledFigure = &allFigures[3];
	if (IsKeyDown(KEY_FIVE)) controlledFigure = &allFigures[4];
	if (IsKeyDown(KEY_SIX)) controlledFigure = &allFigures[5];
}

void cameraControl(Camera3D& camera, float cameraSpeed)
{
	if (IsKeyDown(KEY_RIGHT)) camera.position.x += cameraSpeed;
	if (IsKeyDown(KEY_LEFT)) camera.position.x -= cameraSpeed;
	if (IsKeyDown(KEY_UP)) camera.position.z -= cameraSpeed;
	if (IsKeyDown(KEY_DOWN)) camera.position.z += cameraSpeed;
	if (IsKeyDown(KEY_Q)) camera.position.y += cameraSpeed;
	if (IsKeyDown(KEY_E)) camera.position.y -= cameraSpeed;
}

void Draw(const int figureCount, Figure  allFigures[6], Color  modelColors[6], bool finalCollision, bool broadPhaseCollision, Figure* controlledFigure, std::vector<Vector3>& gridPoints)
{
	for (int i = 0; i < figureCount; i++)
	{
		DrawModelEx(allFigures[i].model, allFigures[i].position, allFigures[i].rotAxis, allFigures[i].rotAngle, allFigures[i].scale, modelColors[i]);
	}

	Color controlledColor = finalCollision ? GOLD : (broadPhaseCollision ? ORANGE : LIME);
	if (controlledFigure)
	{
		DrawAABB(controlledFigure->worldAABB, controlledColor);
	}

	for (int i = 0; i < figureCount; i++)
	{
		if (&allFigures[i] == controlledFigure)
		{
			continue;
		}
		DrawAABB(allFigures[i].worldAABB, DARKGRAY);
	}

	if (broadPhaseCollision)
	{
		for (int idx = 0; idx < (int)gridPoints.size(); idx++)
		{
			DrawPoint3D(gridPoints[idx], finalCollision ? RED : BLUE);
		}
	}

	DrawGrid(20, 1.0f);
	EndMode3D();

	DrawRectangle(10, 10, 260, 180, Fade(SKYBLUE, 0.5f));
	DrawRectangleLines(10, 10, 260, 180, BLUE);

	DrawText("Status:", 20, 20, 20, BLACK);
	if (finalCollision)
	{
		DrawText("GRID COLLISION", 20, 50, 20, RED);
	}
	else if (broadPhaseCollision)
	{
		DrawText("AABB Collision", 20, 50, 20, ORANGE);
	}
	else
	{
		DrawText("No Collision", 20, 50, 20, GREEN);
	}

	DrawText("Camera Controls: Arrows, Q, E", 20, 80, 10, DARKGRAY);
	DrawText("Object Controls: I,J,K,L (Move), U,O (Rotate)", 20, 100, 10, DARKGRAY);
	DrawText("                 Y,H (Scale), R,F (Up/Down)", 20, 120, 10, DARKGRAY);
	DrawText(TextFormat("Controlling: %s", controlledFigure ? controlledFigure->name.c_str() : "None"), 20, 140, 10, DARKBLUE);
	DrawText("Select Object: Keys 1-6", 20, 160, 10, DARKGRAY);

	DrawFPS(GetScreenWidth() - 90, 10);
}

void DrawAABB(MyAABB aabb, Color color)
{
	Vector3 size =
	{
		aabb.max.x - aabb.min.x,
		aabb.max.y - aabb.min.y,
		aabb.max.z - aabb.min.z
	};
	Vector3 center =
	{
		aabb.min.x + size.x * 0.5f,
		aabb.min.y + size.y * 0.5f,
		aabb.min.z + size.z * 0.5f
	};

	DrawCubeWiresV(center, size, color);
}
