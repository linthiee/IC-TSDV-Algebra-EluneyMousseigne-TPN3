#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <float.h>
#include <string>
#include <iostream>

const int gridDivisions = 3;

struct MyAABB
{
    Vector3 min;
    Vector3 max;
};

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
};

MyAABB CalculateLocalAABB(Mesh mesh)
{
    MyAABB aabb;
    if (mesh.vertexCount == 0)
    {
        aabb.min = Vector3Zero();
        aabb.max = Vector3Zero();
        return aabb;
    }

    aabb.min = { mesh.vertices[0], mesh.vertices[1], mesh.vertices[2] };
    aabb.max = aabb.min;

    for (int i = 1; i < mesh.vertexCount; i++)
    {
        Vector3 v =
        {
            mesh.vertices[i * 3 + 0],
            mesh.vertices[i * 3 + 1],
            mesh.vertices[i * 3 + 2]
        };

        aabb.min.x = fminf(aabb.min.x, v.x);
        aabb.min.y = fminf(aabb.min.y, v.y);
        aabb.min.z = fminf(aabb.min.z, v.z);

        aabb.max.x = fmaxf(aabb.max.x, v.x);
        aabb.max.y = fmaxf(aabb.max.y, v.y);
        aabb.max.z = fmaxf(aabb.max.z, v.z);
    }
    return aabb;
}

MyAABB GetUpdatedAABB(MyAABB localBB, Matrix transform)
{
    Vector3 corners[8];
    corners[0] = { localBB.min.x, localBB.min.y, localBB.min.z };
    corners[1] = { localBB.max.x, localBB.min.y, localBB.min.z };
    corners[2] = { localBB.min.x, localBB.max.y, localBB.min.z };
    corners[3] = { localBB.min.x, localBB.min.y, localBB.max.z };
    corners[4] = { localBB.max.x, localBB.max.y, localBB.max.z };
    corners[5] = { localBB.min.x, localBB.max.y, localBB.max.z };
    corners[6] = { localBB.max.x, localBB.min.y, localBB.max.z };
    corners[7] = { localBB.max.x, localBB.max.y, localBB.min.z };

    for (int i = 0; i < 8; i++)
    {
        corners[i] = Vector3Transform(corners[i], transform);
    }

    Vector3 min = corners[0];
    Vector3 max = corners[0];
    for (int i = 1; i < 8; i++)
    {
        min.x = fminf(min.x, corners[i].x);
        min.y = fminf(min.y, corners[i].y);
        min.z = fminf(min.z, corners[i].z);

        max.x = fmaxf(max.x, corners[i].x);
        max.y = fmaxf(max.y, corners[i].y);
        max.z = fmaxf(max.z, corners[i].z);
    }

    return { min, max };
}

bool CheckCollisionAABB(MyAABB aabbA, MyAABB aabbB)
{
    if (aabbA.max.x < aabbB.min.x || aabbA.min.x > aabbB.max.x)
    {
        return false;
    }

    if (aabbA.max.y < aabbB.min.y || aabbA.min.y > aabbB.max.y)
    {
        return false;
    }

    if (aabbA.max.z < aabbB.min.z || aabbA.min.z > aabbB.max.z)
    {
        return false;
    }

    return true;
}

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix)
{
    if (model.meshCount == 0)
    {
        return false;
    }

    Mesh mesh = model.meshes[0];
    
    if (mesh.triangleCount == 0)
    {
        return false;
    }

    Vector3 modelCenter = Vector3Transform(Vector3Zero(), worldMatrix);

    for (int i = 0; i < mesh.triangleCount; i++)
    {
        Vector3 localV1 = { 0, 0, 0 };
        Vector3 localV2 = { 0, 0, 0 };
        Vector3 localV3 = { 0, 0, 0 };

        if (mesh.indices)
        {
            localV1 = { mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 2] };
            localV2 = { mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 2] };
            localV3 = { mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 2] };
        }
        else
        {
            localV1 = { mesh.vertices[(i * 3 + 0) * 3 + 0], mesh.vertices[(i * 3 + 0) * 3 + 1], mesh.vertices[(i * 3 + 0) * 3 + 2] };
            localV2 = { mesh.vertices[(i * 3 + 1) * 3 + 0], mesh.vertices[(i * 3 + 1) * 3 + 1], mesh.vertices[(i * 3 + 1) * 3 + 2] };
            localV3 = { mesh.vertices[(i * 3 + 2) * 3 + 0], mesh.vertices[(i * 3 + 2) * 3 + 1], mesh.vertices[(i * 3 + 2) * 3 + 2] };
        }

        Vector3 v1 = Vector3Transform(localV1, worldMatrix);
        Vector3 v2 = Vector3Transform(localV2, worldMatrix);
        Vector3 v3 = Vector3Transform(localV3, worldMatrix);

        Vector3 edge1 = Vector3Subtract(v2, v1);
        Vector3 edge2 = Vector3Subtract(v3, v1);
        Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));

        Vector3 toCenter = Vector3Subtract(modelCenter, v1);

        if (Vector3DotProduct(normal, toCenter) > 0)
        {
            normal = Vector3Negate(normal);
        }

        Vector3 toPoint = Vector3Subtract(point, v1);
        float dot = Vector3DotProduct(toPoint, normal);

        if (dot > 0.0001f)
        {
            return false; 
        }
    }

    return true;
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

        if (IsKeyDown(KEY_RIGHT)) camera.position.x += cameraSpeed;
        if (IsKeyDown(KEY_LEFT)) camera.position.x -= cameraSpeed;
        if (IsKeyDown(KEY_UP)) camera.position.z -= cameraSpeed;
        if (IsKeyDown(KEY_DOWN)) camera.position.z += cameraSpeed;
        if (IsKeyDown(KEY_Q)) camera.position.y += cameraSpeed;
        if (IsKeyDown(KEY_E)) camera.position.y -= cameraSpeed;

        if (IsKeyDown(KEY_ONE)) controlledFigure = &allFigures[0];
        if (IsKeyDown(KEY_TWO)) controlledFigure = &allFigures[1];
        if (IsKeyDown(KEY_THREE)) controlledFigure = &allFigures[2];
        if (IsKeyDown(KEY_FOUR)) controlledFigure = &allFigures[3];
        if (IsKeyDown(KEY_FIVE)) controlledFigure = &allFigures[4];
        if (IsKeyDown(KEY_SIX)) controlledFigure = &allFigures[5];

        if (controlledFigure)
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

                            bool inA = IsPointInsideMesh(point, controlledFigure->model, controlledFigure->worldMatrix);
                            bool inB = IsPointInsideMesh(point, other->model, other->worldMatrix);

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

        EndDrawing();
    }

    for (int i = 0; i < figureCount; i++)
    {
        UnloadModel(allFigures[i].model);
    }

    CloseWindow();
}