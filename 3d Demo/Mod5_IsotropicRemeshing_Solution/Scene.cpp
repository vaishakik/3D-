#include "Scene.h"

#include <iostream>
#include <fstream>

#include <QString>
#include <QTextStream>
#include <QFileInfo>
#include <QInputDialog>

#include <CGAL/Timer.h>

// For Loop Subdivision
#include <CGAL/Subdivision_method_3.h>

// Include for IO
#include <CGAL/IO/Polyhedron_iostream.h>


// For mesh simplification.
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>

// For Remeshing
#include <CGAL/Polygon_mesh_processing/remesh.h>

#include "render_edges.h"

Scene::Scene()
{
    m_pPolyhedron = NULL;

    // view options
    m_view_polyhedron = true;
}

Scene::~Scene()
{
    delete m_pPolyhedron;
}

int Scene::open(QString filename)
{
    QTextStream cerr(stderr);
    cerr << QString("Opening file \"%1\"\n").arg(filename);
    QApplication::setOverrideCursor(QCursor(::Qt::WaitCursor));

    QFileInfo fileinfo(filename);
    std::ifstream in(filename.toUtf8());

    if(!in || !fileinfo.isFile() || ! fileinfo.isReadable())
    {
        std::cerr << "unable to open file" << std::endl;
        QApplication::restoreOverrideCursor();
        return -1;
    }

    if(m_pPolyhedron != NULL)
        delete m_pPolyhedron;

    // allocate new polyhedron
    m_pPolyhedron = new Polyhedron;
    in >> *m_pPolyhedron;
    if(!in)
    {
        std::cerr << "invalid OFF file" << std::endl;
        QApplication::restoreOverrideCursor();

        delete m_pPolyhedron;
        m_pPolyhedron = NULL;

        return -1;
    }

    QApplication::restoreOverrideCursor();
    return 0;
}

void Scene::update_bbox()
{
    std::cout << "Compute bbox...";
    m_bbox = Bbox();

    if(m_pPolyhedron == NULL)
    {
        std::cout << "failed (no polyhedron)." << std::endl;
        return;
    }

    if(m_pPolyhedron->empty())
    {
        std::cout << "failed (empty polyhedron)." << std::endl;
        return;
    }

    Polyhedron::Point_iterator it = m_pPolyhedron->points_begin();
    m_bbox = (*it).bbox();
    for(; it != m_pPolyhedron->points_end();it++)
        m_bbox = m_bbox + (*it).bbox();
    std::cout << "done (" << m_pPolyhedron->size_of_facets()
        << " facets)" << std::endl;
}

void Scene::draw()
{
    if(m_view_polyhedron)
        render_polyhedron();
}


Vector Scene::normalize(const Vector& v)
{
    return v / std::sqrt(v*v);
}

void Scene::render_polyhedron()
{
    // draw black edges
    if(m_pPolyhedron != NULL)
    {
        ::glDisable(GL_LIGHTING);
        ::glColor3ub(0,0,0);
        ::glLineWidth(1.0f);
        gl_render_edges(*m_pPolyhedron);
    }
}

void Scene::refine_loop()
{
    if(m_pPolyhedron == NULL)
    {
      std::cout << "Load polyhedron first." << std::endl;
      return;
    }

    std::cout << "Loop subdivision...";

    CGAL::Subdivision_method_3::Loop_subdivision(*m_pPolyhedron, 1);

    std::cout << "done (" << m_pPolyhedron->size_of_facets() << " facets)" << std::endl;
}


void Scene::refine_catmull()
{
    if(m_pPolyhedron == NULL)
    {
      std::cout << "Load polyhedron first." << std::endl;
      return;
    }

    std::cout << "Catmull subdivision...";

    CGAL::Subdivision_method_3::CatmullClark_subdivision(*m_pPolyhedron, 1);

    std::cout << "done (" << m_pPolyhedron->size_of_facets() << " facets)" << std::endl;
}


void Scene::simplify_mesh()
{
    if(m_pPolyhedron == NULL)
    {
      std::cout << "Load polyhedron first." << std::endl;
      return;
    }


    CGAL::Surface_mesh_simplification::Count_stop_predicate<Polyhedron> stop(1000);

    // This the actual call to the simplification algorithm.
    // The surface mesh and stop conditions are mandatory arguments.
    // The index maps are needed because the vertices and edges
    // of this surface mesh lack an "id()" field.
    int r = CGAL::Surface_mesh_simplification::edge_collapse
          (*m_pPolyhedron
          ,stop
           ,CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index,*m_pPolyhedron))
                             .halfedge_index_map  (get(CGAL::halfedge_external_index  ,*m_pPolyhedron))
                             .get_cost (CGAL::Surface_mesh_simplification::Edge_length_cost <Polyhedron>())
                          .get_placement(CGAL::Surface_mesh_simplification::Midpoint_placement<Polyhedron>())
       );


    std::cout << "\nFinished...\n" << r << " edges removed.\n"
       << (m_pPolyhedron->size_of_halfedges()/2) << " final edges.\n" ;


    // Let's write the mesh output
    std::cout << "Writing output mesh..." << std::endl;
    std::ofstream os( "out.off" ) ;
    os << *m_pPolyhedron ;
}


void Scene::CreateFaceMap(std::map<face_descriptor, std::size_t>& fi_map)
{
    std::size_t id =0;
    BOOST_FOREACH(face_descriptor f, faces(*m_pPolyhedron))
    {
      fi_map[f]=id++;
    }

}


void Scene::isotropic_remesh()
{
    if( m_pPolyhedron == NULL ) {
      std::cout << "Load polyhedron first." << std::endl;
      return;
    }

    std::map<face_descriptor, std::size_t> fi_map;
    CreateFaceMap(fi_map);

    CGAL::Polygon_mesh_processing::isotropic_remeshing (
            faces(*m_pPolyhedron),
            0.1,
            *m_pPolyhedron,
            CGAL::parameters::face_index_map(
            boost::make_assoc_property_map(fi_map)));


}

void Scene::toggle_view_poyhedron()
{
    m_view_polyhedron = !m_view_polyhedron;
}


// Function for showing stats.
void Scene::ShowStats()
{
    if(m_pPolyhedron == NULL)
    {
        std::cout << "failed (no polyhedron)." << std::endl;
        return;
    }

    std::cout << "Mesh stats are: " << std::endl;

    std::cout << "Is valid " << m_pPolyhedron->is_valid() << std::endl;

    std::cout << "Face count " << m_pPolyhedron->size_of_facets() << std::endl;

    std::cout << "Vertex Count "<< m_pPolyhedron->size_of_vertices() << std::endl;

    std::cout << "Edge Count " << m_pPolyhedron->size_of_halfedges()/2 << std::endl;


    // Check for Euler Poincare Formula for Closed 2-manifold meshes.
    // V + F - E = 2

    int E=m_pPolyhedron->size_of_halfedges()/2;

    int F=m_pPolyhedron->size_of_facets();

    int V=m_pPolyhedron->size_of_vertices();

    if((V+F-E)==2){
        std::cout<<"It follows Euler Equation" << std::endl;
    }else
        std::cout<<"It does not follow Euler Equation" << std::endl;


}
