#include <cnoid/Config>

#ifdef RCLCPP__RCLCPP_HPP_
#define CNOID_GETTEXT_DOMAIN_NAME "CnoidROS2Plugin-" CNOID_VERSION_STRING
#else
#define CNOID_GETTEXT_DOMAIN_NAME "CnoidROSPlugin-" CNOID_VERSION_STRING
#endif

#include <cnoid/GettextUtil>
