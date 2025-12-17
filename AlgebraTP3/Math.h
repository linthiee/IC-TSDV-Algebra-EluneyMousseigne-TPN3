#pragma once
#include "raylib.h"

struct MyAABB
{
    Vector3 min;
    Vector3 max;
};

MyAABB CalculateLocalAABB(Mesh mesh);
MyAABB GetUpdatedAABB(MyAABB localBB, Matrix transform);
bool CheckCollisionAABB(MyAABB aabbA, MyAABB aabbB);

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix);