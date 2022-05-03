// Arcball camera by Eugene Hsu
// Based on 6.839 sample code for rotation code.
// Extended to handle translation (MIDDLE) and scale (RIGHT)

// -*-c++-*-
#ifndef CAMERA_H
#define CAMERA_H

#ifdef WIN32
#include <windows.h>
#endif

#include <vecmath.h>

class Camera
{
public:

    Camera();
    
    typedef enum { NONE, LEFT, MIDDLE, RIGHT } Button;

    // You must call all of the Set*() functions before you use this!
    // I didn't put it into the constructor because it's inconvenient
    // to initialize stuff in my opengl application.
    
    void SetDimensions(int w, int h);
    void SetViewport(int x, int y, int w, int h);
    void SetPerspective(float fovy);

    // Call from whatever UI toolkit
    void MouseClick(Button button, int x, int y);
    void MouseDrag(int x, int y);
    void MouseRelease(int x, int y);

    // Apply viewport, perspective, and modeling
    // use these instead of 
    void ApplyViewport() const;
    
	Matrix4f_ projectionMatrix() const;
	Matrix4f_ viewMatrix() const;

    // Set for relevant vars
    void SetCenter(const Vector3f_& center);
    void SetRotation(const Matrix4f_& rotation);
    void SetDistance(const float distance);

    // Get for relevant vars
    Vector3f_ GetCenter() const { return mCurrentCenter; }
    Matrix4f_ GetRotation() const { return mCurrentRot; }
    float GetDistance() const { return mCurrentDistance; }
    
private:

    // States 
    int     mDimensions[2];
    int     mStartClick[2];
    Button  mButtonState;

    // For rotation
    Matrix4f_ mStartRot;
    Matrix4f_ mCurrentRot;

    // For translation
    float   mPerspective[2];
    int     mViewport[4];
    Vector3f_ mStartCenter;
    Vector3f_ mCurrentCenter;

    // For zoom
    float   mStartDistance;
    float   mCurrentDistance;

    void ArcBallRotation(int x, int y);
    void PlaneTranslation(int x, int y);
    void DistanceZoom(int x, int y);
};

#endif
