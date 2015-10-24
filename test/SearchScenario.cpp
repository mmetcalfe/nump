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
#include "shared/utility/drawing/cairo_drawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

arma::arma_rng::seed_type randomSeed() {
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
    arma::arma_rng::set_seed(seed);
    return seed;
}

numptest::SearchScenario::StateType stateTypeFromYaml(const YAML::Node& yaml) {
    if (yaml.as<std::string>() == "Position") {
        return numptest::SearchScenario::StateType::Position;
    } else if (yaml.as<std::string>() == "PositionBearing") {
        return numptest::SearchScenario::StateType::PositionBearing;
    }

    std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid state type '" << yaml.as<std::string>() << "'." << std::endl;
    exit(1);
}

arma::vec2 vec2FromYaml(const YAML::Node& yaml) {
    return {
            yaml[0].as<double>(),
            yaml[1].as<double>(),
    };
}

arma::vec3 vec3FromYaml(const YAML::Node& yaml) {
    return {
            yaml[0].as<double>(),
            yaml[1].as<double>(),
            yaml[2].as<double>(),
    };
}

arma::mat22 mat22FromYaml(const YAML::Node& yaml) {
    return {
            { yaml[0][0].as<double>(), yaml[0][1].as<double>(), },
            { yaml[1][0].as<double>(), yaml[1][1].as<double>(), },
    };
}

arma::mat33 mat33FromYaml(const YAML::Node& yaml) {
    return {
            { yaml[0][0].as<double>(), yaml[0][1].as<double>(), yaml[0][2].as<double>(), },
            { yaml[1][0].as<double>(), yaml[1][1].as<double>(), yaml[1][2].as<double>(), },
            { yaml[2][0].as<double>(), yaml[2][1].as<double>(), yaml[2][2].as<double>(), },
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

numptest::SearchScenario::Config::KickBox kickBoxConfigFromYaml(const YAML::Node& yaml) {
    numptest::SearchScenario::Config::KickBox kickBoxConfig;

    kickBoxConfig.kickExtent = yaml["kick_extent"].as<double>();
    kickBoxConfig.footWidth = yaml["foot_width"].as<double>();
    kickBoxConfig.footSep = yaml["foot_sep"].as<double>();
    kickBoxConfig.footFrontX = yaml["foot_front_x"].as<double>();

    return kickBoxConfig;
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
    scenario.cfg_.stateType = stateTypeFromYaml(scenarioYaml["state_type"]);

    scenario.cfg_.mapSize = vec2FromYaml(scenarioYaml["map_size"]);
    scenario.cfg_.footprintSize = vec2FromYaml(scenarioYaml["footprint_size"]);

    if (scenario.cfg_.stateType == StateType::Position) {
//        scenario.cfg_.initialState = vec2FromYaml(scenarioYaml["initial_state"]);
//        scenario.cfg_.goalState = vec2FromYaml(scenarioYaml["goal_state"]);
//        scenario.cfg_.initialCovariance = mat22FromYaml(scenarioYaml["initial_covariance"]);
    } else if (scenario.cfg_.stateType == StateType::PositionBearing) {
        scenario.cfg_.initialState = vec3FromYaml(scenarioYaml["initial_state"]);
        scenario.cfg_.goalState = vec3FromYaml(scenarioYaml["goal_state"]);
        scenario.cfg_.initialCovariance = mat33FromYaml(scenarioYaml["initial_covariance"]);
    } else {
        std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid state type '." << std::endl;
        exit(1);
    }


    for (const auto& obsYaml : scenarioYaml["obstacles"]) {
        scenario.cfg_.obstacles.push_back(shapeFromYaml(obsYaml));
    }

    for (const auto& obsYaml : scenarioYaml["measurement_regions"]) {
        scenario.cfg_.measurementRegions.push_back(shapeFromYaml(obsYaml));
    }

    // Load ball approach config:
    scenario.cfg_.ball = shapeFromYaml(scenarioYaml["ball"]);
    scenario.cfg_.targetAngle = scenarioYaml["target_angle"].as<double>();
    scenario.cfg_.targetAngleRange = scenarioYaml["target_angle_range"].as<double>();
    scenario.cfg_.minKickProbability = scenarioYaml["min_kick_probability"].as<double>();
    scenario.cfg_.kbConfig = kickBoxConfigFromYaml(scenarioYaml["kickboxes"]);

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

void numptest::SearchScenario::performRRBTSearch(cairo_t* cr, const std::string& scenario_prefix) {
    // Run RRBT:
     int sampleNum = 0;
     int pointNum = 0;
     auto rrbtTree = nump::RRBT::fromSearchScenario(cfg_, cr,
          [&](const nump::RRBT& rrbt, const nump::RRBT::StateT newState, bool extended) {
              // Make printed output wrap into columns:
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

     writeDataFile(scenario_prefix + "_iteration_times.dat", rrbtTree.iterationTimes);
}

void numptest::SearchScenario::performRRTsSearch(cairo_t* cr, const std::string& scenario_prefix) {
    // Run RRBT:
     int sampleNum = 0;
     int pointNum = 0;
     auto rrtsTree = nump::SearchTree::fromRRTs(cfg_, cr,
          [&](const nump::SearchTree& rrts, const nump::SearchTree::StateT newState, bool extended) {
              // Make printed output wrap into columns:
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
                  drawSearchTree(cr, rrts);
                  cairo_show_page(cr);
                  std::cout << "d" << std::flush;
                  return;
              } else {
                  std::cout << "." << std::flush;
              }
          });
     std::cout << std::endl << "#" << std::endl;

    //  std::cout << "Num nodes: " << rrtsTree.graph.nodes.size() << std::endl;
    //  std::cout << "Num edges: " << rrtsTree.graph.edges.size() << std::endl;
    //  int beliefNodeCount = 0;
    //  for (auto& node : rrtsTree.graph.nodes) {
    //     beliefNodeCount += node->value.beliefNodes.size();
    //  }
    //  std::cout << "Num belief nodes: " << beliefNodeCount << std::endl;

     cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
     drawSearchTree(cr, rrtsTree);
     cairo_show_page(cr);

    //  writeDataFile(scenario_prefix + "_iteration_times.dat", rrtsTree.iterationTimes);
}

void numptest::SearchScenario::execute(const std::string& scenario_prefix) {
    arma::arma_rng::set_seed(cfg_.seed);
    std::cout << "SEED: " << cfg_.seed << std::endl;

    auto ouptut_img_file_name = scenario_prefix + "_searchTests.pdf";
    cairo_surface_t *surface = cairo_pdf_surface_create(ouptut_img_file_name.c_str(), cfg_.canvasSize(0), cfg_.canvasSize(1));
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

    performRRBTSearch(cr, scenario_prefix);
    // performRRTsSearch(cr, scenario_prefix);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}
