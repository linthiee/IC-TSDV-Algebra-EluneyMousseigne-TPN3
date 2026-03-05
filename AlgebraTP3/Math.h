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

void CalculateOBB(Model model, MyOBB& obb, Matrix worldMatrix);
void UpdateOBB(MyOBB& obb, Matrix worldMatrix, Vector3 velocity);

void CalculateUniqueModelPlanes(Model model, std::vector<Plane>& uniquePlanes);
MyAABB CalculateLocalAABB(Mesh mesh);
MyAABB GetUpdatedAABB(MyAABB localBB, Matrix transform);
bool CheckCollisionAABB(MyAABB aabbA, MyAABB aabbB);

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix, std::vector<Plane>& uniquePlanes);

Vector3 operator+(Vector3& vector1, Vector3& vector2);
Vector3 operator-(Vector3& vector1, Vector3& vector2);

float getVectorMagnitude(Vector3 vector);
float getVectorMagnitude(Vector2 vector);

float getVector2Angle(Vector2 v1, Vector2 v2);