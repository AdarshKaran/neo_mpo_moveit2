#pragma once
#include <yaml-cpp/yaml.h>

struct Pose7 {
  double x, y, z;
  double qw, qx, qy, qz;
};
struct BoxDims  { double x,y,z; };
struct CylDims  { double h,r;     };

inline Pose7 parsePose(const YAML::Node& n)
{
  Pose7 p{0, 0, 0, 1, 0, 0, 0}; // defaults

  if (!n) return p;

  if (n.IsSequence()) { // e.g. [0.7, -0.2, 0.7725, 1, 0, 0, 0]
    if (n.size() >= 7) {
      p.x  = n[0].as<double>();
      p.y  = n[1].as<double>();
      p.z  = n[2].as<double>();
      p.qw = n[3].as<double>();
      p.qx = n[4].as<double>();
      p.qy = n[5].as<double>();
      p.qz = n[6].as<double>();
    }
  } else if (n.IsMap()) { // block or flow mapping
    p.x  = n["x"].as<double>();
    p.y  = n["y"].as<double>();
    p.z  = n["z"].as<double>();
    p.qw = n["qw"] ? n["qw"].as<double>() : 1.0;
    p.qx = n["qx"] ? n["qx"].as<double>() : 0.0;
    p.qy = n["qy"] ? n["qy"].as<double>() : 0.0;
    p.qz = n["qz"] ? n["qz"].as<double>() : 0.0;
  }
  return p;
}

inline CylDims parseCylinderDims(const YAML::Node& n, const std::string& id)
{
  if (n.IsSequence()) {
    if (n.size()!=2) throw std::runtime_error(id+" cylinder dims need [h,r]");
    return { n[0].as<double>(), n[1].as<double>() };
  }
  if (n.IsMap())  return { n["height"].as<double>(), n["radius"].as<double>() };
  throw std::runtime_error(id+" cylinder dims must be seq or map");
}

inline BoxDims parseBoxDims(const YAML::Node& n, const std::string& id)
{
  if (n.IsSequence()) {
    if (n.size()!=3) throw std::runtime_error(id+" box dims need [x,y,z]");
    return { n[0].as<double>(), n[1].as<double>(), n[2].as<double>() };
  }
  if (n.IsMap())  return { n["size_x"].as<double>(), n["size_y"].as<double>(), n["size_z"].as<double>() };
  throw std::runtime_error(id+" box dims must be seq or map");
}

// Loads and returns a YAML node, throwing on any failure with a clear message.
inline YAML::Node loadYamlFileOrThrow(const std::string& path)
{
  if (path.empty())
    throw std::runtime_error("YAML path is empty");

  std::ifstream f(path);
  if (!f.good())
    throw std::runtime_error("Failed to open YAML file: " + path);

  try {
    YAML::Node node = YAML::Load(f);
    return node;
  } catch (const YAML::ParserException& e) {
    throw std::runtime_error("YAML parse error in '" + path + "': " + e.what());
  } catch (const std::exception& e) {
    throw std::runtime_error("Unexpected error loading YAML '" + path + "': " + e.what());
  }
}