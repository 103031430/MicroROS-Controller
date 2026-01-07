#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <MicroROS_Transport.h>                         // IMPORTANT: MAKE SURE TO INCLUDE FOR CONNECTION BETWEEN THE ESP AND THE MICRO ROS AGENT TO WORK
#include <sensor_msgs/msg/joy.h>

// Define W5500 Ethernet Chip Pins
#define W5500_CS    14    // CS (Chip Select) PIN
#define W5500_RST   9     // Reset PIN
#define W5500_INT   10    // Interrupt PIN 
#define W5500_MISO  12    // MISO PIN
#define W5500_MOSI  11    // MOSI PIN
#define W5500_SCK   13    // Serial Clock PIN

// Network Configuration
byte esp_mac[] = { 0xDE, 0xAD, 0xAF, 0x91, 0x3E, 0x69 };    // Mac address of ESP32 (Make sure its unique for each ESP32)
IPAddress esp_ip(192, 168, 0, 12);                          // IP address of ESP32   (Make sure its unique for each ESP32)
IPAddress dns(192, 168, 0, 1);                              // DNS Server           (Modify if necessary)
IPAddress gateway(192, 168, 0, 1);                          // Default Gateway      (Modify if necessary)
IPAddress agent_ip(192, 168, 0, 80);                        // IP address of Micro ROS agent   (Modify if necessary)        
size_t agent_port = 8888;                                   // Micro ROS Agent Port Number     (Modify if necessary)

// Define Functions
void error_loop();                                                                            
void HandleConnectionState();
bool CreateEntities();
void DestroyEntities();
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}     // Checks for Errors in Micro ROS Setup
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}              // Checks for Errors in Micro ROS Setup
#define ARRAY_LEN(arr) { (sizeof(arr) / sizeof(arr[0])) }                                     // Calculate the Array Length (Needed for the Int32 Array Publisher)

// Define shared ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rmw_context_t* rmw_context;

// Define Node Name
const char * node_name = "ControllerESP";


// Define MicroROS Subscriber and Publisher entities
  rcl_subscription_t controller;
  sensor_msgs__msg__Joy msg;

// Define Callback functions for the Subscribers
void ControllerCallback(const void * msgin);

// Connection status for the HandleConnection()
enum class ConnectionState {
  Initializing,
  WaitingForAgent,
  Connecting,
  Connected,
  Disconnected
};

ConnectionState connection_state = ConnectionState::Initializing;


// Include the code for startinh up the ethernet chip and initialising the Micro ROS transport
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Ethernet Connection... ");


  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                                   // Initialize SPI with custom pin configuration    
  Ethernet.init(W5500_CS);                                                                  // Select CS PIN and initialize Ethernet chip

  Serial.println("[INIT] Starting micro-ROS node...");
  set_microros_eth_transports(esp_mac, esp_ip, dns, gateway, agent_ip, agent_port);         // IMPORTANT: Start Micro ROS Transport Connection 

  delay(2000);

  connection_state = ConnectionState::WaitingForAgent;

};

void loop() {
  HandleConnectionState();
  delay(1000);

}

// This function handles the connect between Micro ROS inside the ESP and the Micro ROS agent
void HandleConnectionState() {
  switch (connection_state) {
    case ConnectionState::WaitingForAgent:
      // Ping to the Micro ROS agent
      if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent found, establishing connection...");
        connection_state = ConnectionState::Connecting;
      }
      break;

    case ConnectionState::Connecting:
      // Create all micro ROS entities
      if (CreateEntities()) {
        Serial.println("[ROS] Connected and ready!");
        connection_state = ConnectionState::Connected;
      } else {
        Serial.println("[ROS] Connection failed, retrying...");
        connection_state = ConnectionState::WaitingForAgent;
      }
      break;

    case ConnectionState::Connected:
    // If Micro ROS agent gets disconnected...
      if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
        Serial.println("[ROS] Agent disconnected!");
        connection_state = ConnectionState::Disconnected;
      } else {
        Serial.println("heartbeat");                                                      // Use it for testing if the code is working
        
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));                                  // Spins the executor (Important for Subscribers)
      }
      break;

    case ConnectionState::Disconnected:
      DestroyEntities();
      Serial.println("[ROS] Waiting for agent...");
      connection_state = ConnectionState::WaitingForAgent;
      break;

    default:
      break;
  }
}

// This function creates/initialises all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
bool CreateEntities() {

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node object
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // number of subscribers the executor handles is hard coded atm
  
  // ADD ALL YOUR PUBLISHERS AND SUBSCRIBER INITIALISATION HERE
  RCCHECK(rclc_subscription_init_default(
    &controller,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "Joy"
  ));

    msg.axes.data = (float *) malloc(10 * sizeof(float));
    msg.axes.capacity = 10;
    msg.axes.size = 0;

  rclc_executor_add_subscription(&executor, &controller, &msg, &ControllerCallback, ON_NEW_DATA);

  return true;
}



// This function destroys all micro ros entites (Publishers, Subscribers, Executors, Nodes, Support...)
void DestroyEntities() {
    rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    // ADD FUNCTION THAT DESTROYS PUBLISHER AND SUBSCRIBER HERE
    RCCHECK(rcl_subscription_fini(&controller, &node));


    rclc_executor_fini(&executor);                              // Destroy Executors
    RCCHECK(rcl_node_fini(&node));                              // Destroy Node
    rclc_support_fini(&support);                                // Destroy Support
}

// Error handle loop
void error_loop() {
  Serial.println("An error has occured. Restarting...");
  delay(2000);
  ESP.restart();

};

// ========================================= CALLBACK FUNCTIONS ========================================= //
// Create your callback functions here.

void ControllerCallback(const void * msgin)
{
  // Cast received message to used type
  const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;

  size_t size = msg->axes.size;
    
  const float * array_data = msg->axes.data;

  Serial.print("Array size: ");
  Serial.println(size);

  Serial.print("Element ");
  Serial.print(5);
  Serial.print(": ");
  Serial.println(array_data[4]);
  
}