//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#include "SearchScenario.h"
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <armadillo>

#include "nump.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"

using utility::math::geometry::Ellipse;
using shared::utility::drawing::drawSearchTree;
using shared::utility::drawing::drawRRBT;
using shared::utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

arma::arma_rng::seed_type randomSeed() {
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
    arma::arma_rng::set_seed(seed);
    return seed;
}


arma::vec2 stateFromYaml(const YAML::Node& yaml) {
    return {
            yaml[0].as<double>(),
            yaml[1].as<double>(),
    };
}

arma::mat22 stateCovFromYaml(const YAML::Node& yaml) {
    return {
            { yaml[0][0].as<double>(), yaml[0][1].as<double>(), },
            { yaml[1][0].as<double>(), yaml[1][1].as<double>(), },
    };
}

Circle shapeFromYaml(const YAML::Node& yaml) {
    if (yaml["type"].as<std::string>() == "CIRCLE") {
        return {
                {yaml["position"][0].as<double>(), yaml["position"][1].as<double>(),},
                yaml["radius"].as<double>(),
        };
    }

    std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid shape type '" << yaml["type"].as<std::string>() << "'." << std::endl;
    exit(1);
}

numptest::SearchScenario numptest::SearchScenario::fromFile(const std::string &file_path) {

    YAML::Node scenarioYaml = YAML::LoadFile(file_path);

    numptest::SearchScenario scenario;

    // Search settings:
    if (scenarioYaml["seed"].as<std::string>() == "RANDOM") {
        scenario.cfg_.seed = randomSeed();
    } else {
        scenario.cfg_.seed = scenarioYaml["seed"].as<int>();
    }

    scenario.cfg_.numSamples = scenarioYaml["num_samples"].as<int>();


    // Drawing:
    scenario.cfg_.drawPeriod = scenarioYaml["draw_period"].as<int>();
    int canvasWidth = scenarioYaml["canvas_size"][0].as<int>();
    int canvasHeight = scenarioYaml["canvas_size"][1].as<int>();
    scenario.cfg_.canvasSize = {canvasWidth, canvasHeight};

    // Scenario description:
    int mapWidth = scenarioYaml["map_size"][0].as<int>();
    int mapHeight = scenarioYaml["map_size"][1].as<int>();
    scenario.cfg_.mapSize = {mapWidth, mapHeight};

    scenario.cfg_.initialState = stateFromYaml(scenarioYaml["initial_state"]);
    scenario.cfg_.goalState = stateFromYaml(scenarioYaml["goal_state"]);
    scenario.cfg_.initialCovariance = stateCovFromYaml(scenarioYaml["initial_covariance"]);

    for (const auto& obsYaml : scenarioYaml["obstacles"]) {
        scenario.cfg_.obstacles.push_back(shapeFromYaml(obsYaml));
    }

    for (const auto& obsYaml : scenarioYaml["measurement_regions"]) {
        scenario.cfg_.measurementRegions.push_back(shapeFromYaml(obsYaml));
    }

    return scenario;
}

void numptest::SearchScenario::execute() {
    arma::arma_rng::set_seed(cfg_.seed);
    std::cout << "SEED: " << cfg_.seed << std::endl;

   cairo_surface_t *surface = cairo_pdf_surface_create("searchTests.pdf", cfg_.canvasSize(0), cfg_.canvasSize(1));
   cairo_t *cr = cairo_create(surface);

    cairo_scale(cr, cfg_.canvasSize(0), cfg_.canvasSize(1));
//    double centeredFrac = 0.85;
//    double borderSize = 0.5 * (1 - centeredFrac) / centeredFrac;
//    cairo_scale(cr, centeredFrac, centeredFrac);
//    cairo_translate(cr, borderSize, borderSize);


   // Run RRBT:
   int pointNum = 0;
   auto rrbtTree = nump::RRBT::fromRRBT(cr
       , cfg_.initialState
       , cfg_.initialCovariance
       , cfg_.goalState
       , cfg_.numSamples
       , cfg_.obstacles
       , cfg_.measurementRegions
       , [&](const nump::RRBT& rrbt, const nump::RRBT::StateT newState, bool extended) {

             if (!extended) {
                 std::cout << "c" << std::flush;
                 return;
             }

             pointNum++;

             if (pointNum % cfg_.drawPeriod == 0) {
                 cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
                 drawRRBT(cr, rrbt);
                 cairo_show_page(cr);
                 std::cout << "d" << std::flush;
                 return;
             } else {
                 std::cout << "." << std::flush;
             }
         });
   std::cout << "#" << std::endl;

   std::cout << "Num nodes: " << rrbtTree.graph.nodes.size() << std::endl;
   std::cout << "Num edges: " << rrbtTree.graph.edges.size() << std::endl;
   int beliefNodeCount = 0;
   for (auto& node : rrbtTree.graph.nodes) {
       beliefNodeCount += node->value.beliefNodes.size();
   }
   std::cout << "Num belief nodes: " << beliefNodeCount << std::endl;

   cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
   drawRRBT(cr, rrbtTree);
   cairo_show_page(cr);

   std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

   // Clean up:
   cairo_destroy(cr);
   cairo_surface_destroy(surface);
}
