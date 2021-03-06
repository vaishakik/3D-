#ifndef SCENE_H
#define SCENE_H

#include <QtOpenGL/qgl.h>
#include <iostream>
#include <cmath>

#include "types.h"

class Scene
{
public:
    Scene();
    ~Scene();
public:
    // types
    typedef CGAL::Bbox_3 Bbox;

public:
    void update_bbox();
    Bbox bbox() { return m_bbox; }

private:
    // member data
    Bbox m_bbox;
    Polyhedron *m_pPolyhedron;

    // view options
    bool m_view_polyhedron;

public:
    // file menu
    int open(QString filename);

    // toggle view options
    void toggle_view_poyhedron();

    // algorithms
    Vector normalize(const Vector& v);

    void refine_loop();

    void refine_catmull();

    void simplify_mesh();

    // rendering
    void draw(); 
    void render_polyhedron();

    //For showing mesh stats.
    void ShowStats();


}; // end class Scene


#endif // SCENE_H
