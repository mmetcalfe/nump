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
    scenario.cfg_.rrbtAppendRejectCovThreshold = scenarioYaml["rrbt_append_reject_cov_threshold"].as<double>();
    scenario.cfg_.rrbtAppendRejectCostThreshold = scenarioYaml["rrbt_append_reject_cost_threshold"].as<double>();


    // Drawing:
    scenario.cfg_.drawPeriod = scenarioYaml["draw_period"].as<int>();
    int canvasWidth = scenarioYaml["canvas_size"][0].as<int>();
    int canvasHeight = scenarioYaml["canvas_size"][1].as<int>();
    scenario.cfg_.canvasSize = {canvasWidth, canvasHeight};

    // Scenario description:
    double mapWidth = scenarioYaml["map_size"][0].as<double>();
    double mapHeight = scenarioYaml["map_size"][1].as<double>();
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

void writeDataFile(std::string fname, const std::vector<double>& values) {
//    std::ofstream fs(fname, std::ios::out | std::ios::app);
    std::ofstream fs(fname, std::ios::out);
    for (int i = 0; i < values.size(); i++) {
        fs << (i+1) << " " << values[i] << std::endl;
    }
    fs << std::endl;
    fs.close();
}

void numptest::SearchScenario::execute() {
    arma::arma_rng::set_seed(cfg_.seed);
    std::cout << "SEED: " << cfg_.seed << std::endl;

    cairo_surface_t *surface = cairo_pdf_surface_create("searchTests.pdf", cfg_.canvasSize(0), cfg_.canvasSize(1));
    cairo_t *cr = cairo_create(surface);

    double canvasAspect = cfg_.canvasSize(0) / cfg_.canvasSize(1);
    double mapAspect = cfg_.mapSize(0) / cfg_.mapSize(1);

    if (mapAspect > canvasAspect) {
        double drawingScale = cfg_.canvasSize(0) / cfg_.mapSize(0);
        cairo_scale(cr, drawingScale, drawingScale);
    } else {
        double drawingScale = cfg_.canvasSize(1) / cfg_.mapSize(1);
        cairo_scale(cr, drawingScale, drawingScale);
    }

//    double centeredFrac = 0.85;
//    double borderSize = 0.5 * (1 - centeredFrac) / centeredFrac;
//    cairo_scale(cr, centeredFrac, centeredFrac);
//    cairo_translate(cr, borderSize, borderSize);


   // Run RRBT:
    int sampleNum = 0;
    int pointNum = 0;
    auto rrbtTree = nump::RRBT::fromSearchScenario(cfg_, cr,
         [&](const nump::RRBT& rrbt, const nump::RRBT::StateT newState, bool extended) {
             if (sampleNum % 50 == 0) {
                 std::cout << std::endl;
             }
             sampleNum++;

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
    std::cout << std::endl << "#" << std::endl;

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

    writeDataFile("iteration_times.dat", rrbtTree.iterationTimes);

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}
