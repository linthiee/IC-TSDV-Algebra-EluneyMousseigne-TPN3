#pragma once
#include "raylib.h"
#include <vector>

struct MyAABB
{
    Vector3 min;
    Vector3 max;
};

struct MyOBB
{
    Vector3 center;
    Vector3 localAxes[3];
	Vector3 halfSizes;
};

struct Plane
{
    Vector3 normal;
    float d;
};

void CalculateOBB(Model model, MyOBB& obb, Vector3 rotAxis, float rotAngle);
void CalculateUniqueModelNormal(Model model, std::vector<Plane>& uniqueNormals);
MyAABB CalculateLocalAABB(Mesh mesh);
MyAABB GetUpdatedAABB(MyAABB localBB, Matrix transform);
bool CheckCollisionAABB(MyAABB aabbA, MyAABB aabbB);

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix, std::vector<Plane>& uniqueNormals);