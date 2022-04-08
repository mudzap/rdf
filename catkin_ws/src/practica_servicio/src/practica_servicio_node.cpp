#include <ros/ros.h>
#include <def_servicios/Lookup.h>
#include <string>

#define RES_ARR_SIZE 5
// Constantes
std::string mensajes[RES_ARR_SIZE] = {
  "Hola", "Mundo", "Soy", "Un", "Tutorial"
};

// Callback de servidor
bool lookup_msg(def_servicios::Lookup::Request &req,
		def_servicios::Lookup::Response &res) {
  if (req.id >= RES_ARR_SIZE) {
    res.error_code = def_servicios::LookupResponse::INVALID_ID;
    res.response = "";
  } else {
    res.error_code = def_servicios::LookupResponse::OK;
    res.response = mensajes[req.id];
  }
  // Necesario retornar true cuando se completa el servicio
  return true;  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nombre_de_nodo");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("nombre_de_servicio", lookup_msg);
  ROS_INFO("Servicio listo");
  ros::spin();

  return 0;
}
