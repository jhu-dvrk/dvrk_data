#include <dvrk_data/config.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace sv {

bool Config::load_from_file(const std::string& path, Json::Value& root) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON: " << path << std::endl;
        return false;
    }

    try {
        ifs >> root;
    } catch (const std::exception& e) {
        std::cerr << "JSON parse error in " << path << ": " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Config::check_type(const Json::Value& root, const std::string& expected_type, const std::string& path) {
    if (!root.isMember("type")) {
        std::cerr << "Error: JSON file '" << path << "' is missing the \"type\" field. "
                  << "Expected \"" << expected_type << "\"." << std::endl;
        return false;
    }

    const std::string actual_type = root["type"].asString();
    if (actual_type != expected_type) {
        std::cerr << "Error: Incompatible JSON type in '" << path << "'. "
                  << "Found \"" << actual_type << "\", but expected \"" << expected_type << "\"."
                  << std::endl;
        return false;
    }
    return true;
}

AppConfig Config::parse_app_config(const Json::Value& root) {
    AppConfig cfg;

    auto parse_color = [](const Json::Value& node) {
        ColorAdjustment color;
        if (node.isMember("brightness")) color.brightness = node["brightness"].asDouble();
        if (node.isMember("contrast"))   color.contrast   = node["contrast"].asDouble();
        if (node.isMember("saturation")) color.saturation = node["saturation"].asDouble();
        if (node.isMember("hue"))        color.hue        = node["hue"].asDouble();
        return color;
    };

    auto parse_endpoint = [](const Json::Value& node) {
        SourceConfig endpoint;
        if (!node.isObject()) return endpoint;
        if (node.isMember("gst_input") && node["gst_input"].isString())
            endpoint.gst_input = node["gst_input"].asString();
        if (node.isMember("gst_output") && node["gst_output"].isString()) {
            endpoint.gst_output = node["gst_output"].asString();
            endpoint.gst_output_specified = true;
        }
        return endpoint;
    };

    // ── Root-level fields ──────────────────────────────────────────────────────
    cfg.name = root.get("name", "dvrk_display").asString();
    if (cfg.name.empty()) cfg.name = "dvrk_display";

    cfg.dvrk_console_namespace = root.get("dvrk_console_namespace", "console").asString();
    if (cfg.dvrk_console_namespace.empty()) cfg.dvrk_console_namespace = "console";

    cfg.overlay_alpha = root.get("overlay_alpha", 0.7).asDouble();
    cfg.preserve_size = root.get("preserve_size", true).asBool();

    if (root.isMember("display_horizontal_offset_px"))
        cfg.display_horizontal_offset_px = root["display_horizontal_offset_px"].asInt();

    if (root.isMember("sinks") && root["sinks"].isArray()) {
        for (const auto& item : root["sinks"]) {
            if (!item.isString()) continue;
            const std::string sink_type = item.asString();
            cfg.sinks.push_back(sink_type);
            if (sink_type == "glimage") {
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
            } else if (sink_type == "glimages") {
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
            }
        }
    }

    const Json::Value empty_object(Json::objectValue);
    const Json::Value& cam =
        (root.isMember("camera") && root["camera"].isObject()) ? root["camera"] : empty_object;

    if (root.isMember("gst_input") && root["gst_input"].isString())
        cfg.stereo.gst_input = root["gst_input"].asString();
    if (root.isMember("gst_output") && root["gst_output"].isString()) {
        cfg.stereo.gst_output = root["gst_output"].asString();
        cfg.stereo.gst_output_specified = true;
    }
    if (root.isMember("eye_size") && root["eye_size"].isObject()) {
        const Json::Value& sz = root["eye_size"];
        if (sz.isMember("width")) cfg.original_width = sz["width"].asInt();
        if (sz.isMember("height")) cfg.original_height = sz["height"].asInt();
    }

    if (cam.isMember("size") && cam["size"].isObject()) {
        const Json::Value& sz = cam["size"];
        if (!sz.isMember("width") || !sz.isMember("height"))
            throw std::runtime_error("Configuration error: 'camera.size' must define both 'width' and 'height'.");
        cfg.original_width  = sz["width"].asInt();
        cfg.original_height = sz["height"].asInt();
    }

    if (cam.isMember("left") && cam["left"].isObject()) {
        const Json::Value& lft = cam["left"];
        cfg.left = parse_endpoint(lft);
        if (lft.isMember("color") && lft["color"].isObject())
            cfg.left_color = parse_color(lft["color"]);
    }

    if (cam.isMember("right") && cam["right"].isObject()) {
        const Json::Value& rgt = cam["right"];
        cfg.right = parse_endpoint(rgt);
        if (rgt.isMember("color") && rgt["color"].isObject())
            cfg.right_color = parse_color(rgt["color"]);
    }

    if (cam.isMember("crop") && cam["crop"].isObject()) {
        const Json::Value& crop = cam["crop"];
        if (crop.isMember("width"))  cfg.crop_width  = crop["width"].asInt();
        if (crop.isMember("height")) cfg.crop_height = crop["height"].asInt();
    }

    if (cam.isMember("alignment") && cam["alignment"].isObject()) {
        const Json::Value& align = cam["alignment"];
        if (align.isMember("horizontal_shift_px"))
            cfg.horizontal_shift_px = align["horizontal_shift_px"].asInt();
        if (align.isMember("vertical_shift_px"))
            cfg.vertical_shift_px = align["vertical_shift_px"].asInt();
    }

    if (cfg.crop_width <= 0) {
        cfg.crop_width = cfg.original_width;
    }
    if (cfg.crop_height <= 0) {
        cfg.crop_height = cfg.original_height;
    }

    if (root.isMember("pip_gst_inputs") && root["pip_gst_inputs"].isObject()) {
        const Json::Value& es = root["pip_gst_inputs"];
        if (es.isMember("monos") && es["monos"].isArray()) {
            for (const auto& item : es["monos"]) {
                if (item.isObject() && item["gst_input"].isString() &&
                    !item["gst_input"].asString().empty()) {
                    cfg.pip_gst_inputs.monos.push_back(item["gst_input"].asString());
                }
            }
        }
        
        if (es.isMember("stereos") && es["stereos"].isArray()) {
            for (const auto& item : es["stereos"]) {
                if (!item.isObject()) continue;
                if (item.isMember("left") && item.isMember("right") &&
                    item["left"].isObject() && item["right"].isObject() &&
                    item["left"]["gst_input"].isString() &&
                    item["right"]["gst_input"].isString()) {
                    StereoExtraStream ses;
                    ses.left = item["left"]["gst_input"].asString();
                    ses.right = item["right"]["gst_input"].asString();
                    if (!ses.left.empty() && !ses.right.empty()) {
                        cfg.pip_gst_inputs.stereos.push_back(ses);
                    }
                }
            }
        }

        // Enforce maximum of 2 total streams (monos + stereos)
        int total_streams = cfg.pip_gst_inputs.monos.size() + cfg.pip_gst_inputs.stereos.size();
        if (total_streams > 2) {
            std::cerr << "Warning: Maximum of 2 pip_gst_inputs entries allowed. Truncating." << std::endl;
            while (cfg.pip_gst_inputs.monos.size() + cfg.pip_gst_inputs.stereos.size() > 2) {
                if (!cfg.pip_gst_inputs.stereos.empty()) {
                    cfg.pip_gst_inputs.stereos.pop_back();
                } else {
                    cfg.pip_gst_inputs.monos.pop_back();
                }
            }
        }
        if (es.isMember("scale")) {
            const double s = es["scale"].asDouble();
            cfg.pip_gst_inputs.scale = std::max(0.01, std::min(0.99, s));
        }
    }

    if (root.isMember("ar") && root["ar"].isObject()) {
        const Json::Value& ar = root["ar"];
        cfg.ar.enabled = ar.get("enabled", true).asBool();
        if (ar.isMember("left")) cfg.ar.left = parse_endpoint(ar["left"]);
        if (ar.isMember("right")) cfg.ar.right = parse_endpoint(ar["right"]);
        if (ar.isMember("color_key") && ar["color_key"].isArray() && ar["color_key"].size() == 3) {
            cfg.ar.use_color_key = true;
            cfg.ar.color_key_r = ar["color_key"][0].asInt();
            cfg.ar.color_key_g = ar["color_key"][1].asInt();
            cfg.ar.color_key_b = ar["color_key"][2].asInt();
        }
    }

    return cfg;
}

}  // namespace sv
