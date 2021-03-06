#ifndef PCA_DEMO_TYPES_H
#define PCA_DEMO_TYPES_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <boost/iterator/transform_iterator.hpp> 

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Line_3 Line;
typedef Kernel::Point_3 Point;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Segment_3 Segment;
typedef Kernel::Triangle_3 Triangle;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;

#endif // PCA_DEMO_TYPES_H
