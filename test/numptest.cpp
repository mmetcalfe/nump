//#include <iostream>
//#include <cairo/cairo.h>
//#include <cairo/cairo-pdf.h>
//#include <math.h>
//#include <armadillo>
//#include "nump.h"
//#include "shared/utility/drawing/SearchTreeDrawing.h"
//#include "shared/utility/math/geometry/Ellipse.h"
//#include "shared/utility/math/geometry/intersection/Intersection.h"
//#include "tests.h"
//
//using utility::math::geometry::Ellipse;
//using shared::utility::drawing::drawSearchTree;
//using shared::utility::drawing::drawRRBT;
//using shared::utility::drawing::fillCircle;
//using nump::math::Transform2D;
//using nump::math::Circle;
//
//arma::arma_rng::seed_type randomSeed() {
//    arma::arma_rng::set_seed_random();
//    arma::vec randvec = arma::randu(1);
//    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
//    arma::arma_rng::set_seed(seed);
//    return seed;
//}
//
//int main() {
//    // // Run the tests:
//    covarianceComparisonTests();
//    trajectoryTests();
//    intersectionTests();
//    propogateTests();
//
//
//    arma::ivec2 surfaceDimensions = {500, 500};
//    cairo_surface_t *surface = cairo_pdf_surface_create("searchTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
//    cairo_t *cr = cairo_create(surface);
//
//    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));
//    double centeredFrac = 0.85;
//    double borderSize = 0.5 * (1 - centeredFrac) / centeredFrac;
//    cairo_scale(cr, centeredFrac, centeredFrac);
//    cairo_translate(cr, borderSize, borderSize);
//
//    // Get random seed:
//    // int seed = 114013460; // randomSeed();
//    int seed = 1140; // randomSeed();
//    arma::arma_rng::set_seed(seed);
//    std::cout << "SEED: " << seed << std::endl;
//
//    // Setup problem:
////    Transform2D start = {0, 0, 0};
////    Transform2D goal  = {1, 1, 0};
//    arma::vec2 start = {0, 0.6};
//    arma::vec2 goal  = {1, 0};
//
//    int numPoints = 300;
//    std::vector<nump::math::Circle> obstacles;
//    std::vector<nump::math::Circle> measurementRegions;
////    obstacles.push_back({{0.6, -0.9}, 1.0});
//////    obstacles.push_back({{0.2, 0.2}, 0.1});
//    // obstacles.push_back({{0.5, 0.5}, 0.1});
//////    obstacles.push_back({{0.8, 0.5}, 0.25});
////    obstacles.push_back({{0.4, 1.5}, 0.7});
//
//
//    obstacles.push_back({{0.0, 0.3}, 0.1});
//    obstacles.push_back({{0.1, 0.3}, 0.1});
//    obstacles.push_back({{0.2, 0.3}, 0.1});
//    obstacles.push_back({{0.3, 0.3}, 0.1});
//    obstacles.push_back({{0.7, 0.3}, 0.1});
//    obstacles.push_back({{0.8, 0.3}, 0.1});
//    obstacles.push_back({{0.9, 0.3}, 0.1});
//    obstacles.push_back({{1.0, 0.3}, 0.1});
//    measurementRegions.push_back({{0.0, 1.0}, 0.2});
//    measurementRegions.push_back({{0.1, 1.0}, 0.2});
//    measurementRegions.push_back({{0.2, 1.0}, 0.2});
//    measurementRegions.push_back({{0.3, 1.0}, 0.2});
//    measurementRegions.push_back({{0.4, 1.0}, 0.2});
//    measurementRegions.push_back({{0.5, 1.0}, 0.2});
//    measurementRegions.push_back({{0.6, 1.0}, 0.2});
//    measurementRegions.push_back({{0.7, 1.0}, 0.2});
//    measurementRegions.push_back({{0.8, 1.0}, 0.2});
//    measurementRegions.push_back({{0.9, 1.0}, 0.2});
//    measurementRegions.push_back({{1.0, 1.0}, 0.2});
//
//
//    // for (auto& obs : obstacles) {
//    //     nump::math::Circle reg = {obs.centre, obs.radius + 0.3};
//    //     measurementRegions.push_back(reg);
//    // }
////
////    // Run RRT:
////    arma::arma_rng::set_seed(seed);
////    auto rrtTree = nump::SearchTree::fromRRT(cr, start, goal, numPoints, obstacles);
////
////    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
////    drawSearchTree(cr, rrtTree);
////    cairo_show_page(cr);
////    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//////    cairo_push_group(cr);
////
////    // Run RRT*:
////    arma::arma_rng::set_seed(seed);
////    auto rrtsTree = nump::SearchTree::fromRRTs(cr, start, goal, numPoints, obstacles, [cr](const nump::SearchTree& tree, const nump::SearchTree::StateT newState, bool extended){
//////        if (!extended) {
//////            return;
//////        }
//////        cairo_pattern_t *group = cairo_pop_group(cr);
//////
////////        fillCircle(cr, newState.rows(0,1), 0.025, {1, 0, 0});
//////        drawSearchTree(cr, tree);
//////
//////        cairo_set_source(cr, group);
//////        cairo_paint(cr);
//////        cairo_pattern_destroy(group);
//////
//////        cairo_show_page(cr);
//////        cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//////        cairo_push_group(cr);
////    });
////
//////    cairo_pop_group_to_source(cr);
////
////    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
////    drawSearchTree(cr, rrtsTree);
////    cairo_show_page(cr);
//
//
//    // Run RRBT:
//    arma::arma_rng::set_seed(seed);
//    nump::RRBT::StateCovT initCov = arma::diagmat(arma::vec({0.001, 0.001}));
//    initCov *= 4;
////    nump::RRBT::StateCovT initCov = arma::mat({{0.05, 0.01},{0.01,0.05}});
//    int pointNum = 0;
//    int drawPeriod = 100;
//    auto rrbtTree = nump::RRBT::fromRRBT(cr
//        , start
//        , initCov
//        , goal
//        , numPoints
//        , obstacles
//        , measurementRegions
//        , [&](const nump::RRBT& rrbt, const nump::RRBT::StateT newState, bool extended) {
//
//              if (!extended) {
//                  std::cout << "c" << std::flush;
//                  return;
//              }
//
//              pointNum++;
//
//              if (pointNum % drawPeriod == 0) {
//                  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//                  drawRRBT(cr, rrbt);
//                  cairo_show_page(cr);
//                  std::cout << "d" << std::flush;
//                  return;
//              } else {
//                  std::cout << "." << std::flush;
//              }
//          });
//    std::cout << "#" << std::endl;
//
//    std::cout << "Num nodes: " << rrbtTree.graph.nodes.size() << std::endl;
//    std::cout << "Num edges: " << rrbtTree.graph.edges.size() << std::endl;
//    int beliefNodeCount = 0;
//    for (auto& node : rrbtTree.graph.nodes) {
//        beliefNodeCount += node->value.beliefNodes.size();
//    }
//    std::cout << "Num belief nodes: " << beliefNodeCount << std::endl;
//
//    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//    drawRRBT(cr, rrbtTree);
//    cairo_show_page(cr);
//
//    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
//
//    // Clean up:
//    cairo_destroy(cr);
//    cairo_surface_destroy(surface);
//    return 0;
//}
