/*************************************************************
 * General-purpose template for basic ROS nodes
 * by Giulio Fontana - AIRLab (Politecnico di Milano)
 * version 0.82 - 2014/01/14
 *************************************************************
 *
 * By using this template you can define a node that performs 
 * any combination of the following types of activities
 * (multiple instances of the same type of activity are
 * allowed):
 * 1- publish messages to a ROS topic
 * 2- request a service from ROS servers as a ROS client
 * 3- act as a ROS server, providing a service to ROS clients
 * 4- execute a task whenever a new message is published on a
 *    specific ROS topic
 * 5- manage a timeout and execute a task whenever it expires
 * 6- execute a task periodically
 *
 * You can choose wether the node should run continously (e.g.,
 * executing tasks as soon as the need emerges) or run
 * periodically (e.g., "awakening" periodically to process
 * pending requests for the execution of tasks). 
 * Not all types of activities are compatible with both modes:
 * --activities of types 1 to 5 are suitable for both continuous
     and periodic running;
 * --activities of type 6 are suitable for periodic running only
 *   (the task is executed once for each run).
 *
 * Please note that also for continuously running nodes it is
 * possible to define periodic tasks: in that case, they have to be
 * defined as activities of type 5, triggered by a timeout
 * which is reset at every execution of the task.
 *
 * It is also important to note that, in this context, "execute a
 * task" actually means "call a function" (including the case where
 * the function is a class method). ROS provides ways to issue
 * such function calls, but what the called function actually does
 * can interfere with the desired behaviour of the node in terms of
 * task execution. It's up to the programmer who writes the function
 * to avoid potential misbehaviour.
 * For instance: if your node is expected to execute a task
 * periodically, calling function MyFunc every 25ms, it is your
 * responsibility to ensure that the execution of MyFunc always ends
 * within 25ms, in order to let the next call to MyFunc be issued
 * on time. For instance, you should write the code of MyFunc in
 * such a way that it cannot get stuck (e.g., waiting for data).
 * Whatever your functions (such as MyFunc) do, once one of them has
 * been called by ROS, ROS will do nothing to take back control. It
 * will simply wait for the end of the execution of the function,
 * before resuming its management of the node's tasks.
 *
 * Important: this template should be considered only as a tool for
 * quick setup of simple ROS nodes! The way it does things should
 * not be considered as anything more than one possible
 * implementation. Feel free to experiment and make different
 * choices, according to your own requirements.
 *************************************************************/
 
/*************************************************************
 * A NOTE ABOUT HOW ROS MANAGES EVENT-TRIGGERED ACTIVITIES
 *************************************************************
 * Some of the types of activities described by this template are
 * event-triggered, i.e.:
 * --activities of type 3, triggered by a request by a client;
 * --activities of type 4, triggered by the publication of a
 *   message;
 * --activities of type 5, triggered by the expiry of a timeout.
 *
 * Such triggered activities are managed by ROS through calls to 
 * functions ("callback functions") written by the user. Callbacks
 * (i.e., calls to callback functions) are managed by ROS, and
 * they are queued so that the node is able to process them when
 * it is ready. This allows periodically run nodes to perform
 * event-triggererd activities: callbacks are processed at the
 * first run of the node. ROS takes care of the queuing.
 *
 * Please note that if a timeout expires while a node is not
 * running, the event will only be acknowledged by the node at its
 * next run, whenever it is. So be *very* careful if you plan to
 * manage safety timeouts with periodic nodes.
 *
 * In the special case of activities of type 4, queue length
 * (number of messages that are retained before the oldest gets
 * discarded every time a new one is published) is chosen by the
 * programmer, and is critical for the correct processing of such
 * events. Therefore queue length should be chosen with care,
 * especially if the node is run periodically, to avoid overflow.
 *
 * It should be noted that nothing prevents nodes acting as ROS
 * servers from being run periodically. If a client issues a
 * request to a server when it is inactive (waiting for its next
 * run), the request is queued by ROS and the client will not
 * get the response until the next run of the server. The client
 * client will have to wait, but does not get a "server not
 * responding" error.
 *************************************************************/
  

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/*************************************************************
 * default value -in seconds- for the period between subsequent
 * runs of the node. It is used only if the node is run
 * periodically AND if the actual value of the period is not
 * retrieved from the ROS parameter server by function
 * ROSnode::Prepare */
 
#define NAME_OF_THIS_NODE "my_ROS_node"
/*************************************************************
 * Substitute "my_ROS_node" with the actual name of the node
 * you are defining, between double quotes.
 * After you compile this file, an executable is created in the
 * /bin subfolder of your package directory.
 * Whenever such executable is run with rosrun, a node is
 * added to the running ROS system, the name of which is the
 * one specified here. If the node is run with roslaunch,
 * the actual name of the node can be specified by the <node>
 * statement in the launchfile, so it may differ from the one
 * specified here. Actually, as ROS does not allow multiple 
 * nodes having the same name to run at the same time, changing
 * the name of the node with the launchfile is a good idea to
 * prevent conflicts, whenever there is the possibility that
 * multiple instances of the node will be running at the same
 * time. */

// #include "name_of_the_server_package/name_of_the_service.h"
/*************************************************************
 * Uncomment if your node accesses a ROS service called 
 * name_of_the_service, provided by a server which is part of a
 * ROS package called name_of_the_server_package, or if yours is 
 * the node which provides such service.
 *
 * Remember that the inputs and outputs of each service provided
 * by your node must be defined by a dedicated .srv file in your
 * ROS package, and that the CMakeLists.txt file of your package
 * has to include the rosbuild_genmsg() line to enable generation
 * of the relevant files during compilation.
 */

// #include "name_of_the_msg_package/name_of_the_msg_type.h"
/*************************************************************
 * Uncomment if your node publishes or receives ROS messages.
 * name_of_the_msg_type must be substituted with the name of
 * the type of ROS message that your node uses; 
 * name_of_the_msg_package must be substituted with the name of
 * the ROS package where the .msg file that defines the internal
 * structure of such type of message is located.
 * A package which defines many common ROS message types is
 * std_msgs: a list of the message types it provides is here:
 * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
 *
 * If your node uses more than one type of message, one #include
 * directive for each one of those types must be used.
 *
 * Remember that the internal structure of the messages used by
 * your node must be defined by a dedicated .msg file. Such file
 * can be part of a standard ROS package or be part of your own
 * package. In the second case, the CMakeLists.txt file of your 
 * package should include the rosbuild_gensrv() line, to enable
 * generation of the relevant .h file during compilation.
 */
 
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode
/* this class template includes all the elements needed to define
 * a basic ROS node */
{
  private: 
    ros::NodeHandle Handle;
    
    // ros::Publisher Publisher;
    /*********************************************************
     * Uncomment if your node publishes messages to a topic.
     *
     * Add one similar variable declaration for each additional
     * topic that your node publishes on.  
     */
    
    
    // void PeriodicTask(void);
    /*********************************************************
     * Uncomment if your node is run periodically and needs to
     * execute a task each time it is run. Such task is
     * represented by method PeriodicTask.
     */
     
    
    
  public:
    double RunPeriod;
    /* if the node is run periodically, this is the period -in 
     * seconds- between the start of one execution of the node 
     * and the start of the next one */
    
    void Prepare(void);
    /* prepares the node for running: all the preparatory tasks
     * that the node has to perform (both due to the interaction
     * with the ROS system and to your particular application)
     * should be performed by this method */
    
    void RunContinuously(void);
    /* used to run the node if it has to run continuously.
     * WARNING: this method and RunPeriodically should never be
     * both called for the same node. */
    
    void RunPeriodically(float Period);
    /* used to run the node if it has to run periodically; Period
     * is the distance in time between runs, in seconds.
     * WARNING: this method and RunContinuously should never be
     * both called for the same node. */
    
    void Shutdown(void);
    /* performs all the activities that the node is required to 
     * perform immediately before it shuts down
     * NOTE: all the elements of the node that are provided by ROS
     * do not require you to do anything to shut down a node; this 
     * method is provided as a container for code that your own
     * node may be required to run in order to shut down its
     * activities in a clean way. */
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;

  std::string FullParamName;
  /* full (i.e. fully resolved starting from namespace /) name of
   * a parameter to be retrieved from the ROS parameter server. 
   * Actually used only if your node accesses the ROS parameter
   * server to retrieve parameter values (how to do this is
   * explained below). */
  
  /***********************************************************
   * RETRIEVING THE VALUE OF RunPeriod FROM THE PARAMETER SERVER
   * [a generic version of the following code, suitable for
   *  retrieving generic parameters, will be provided later]
   *
   * If your node runs periodically, you have to assign a 
   * value to variable RunPeriod. If you do nothing, RunPeriod
   * takes value RUN_PERIOD_DEFAULT, as defined at the beginning
   * of this file.
   * If you want to modify the value of RunPeriod, you can either
   * change RUN_PERIOD_DEFAULT (but you will have to recompile 
   * this file after the change), or you can adopt a more
   * sophisticated approach that does not require recompiling.
   * In fact, provided that you define parameter
   * 
   * /name_of_the_namespace/NAME_OF_THIS_NODE/run_period
   *
   * (of type double) in the ROS parameter server, and that you
   * assign the desired value to it, you can have the node read the
   * value of RunPeriod from the parameter server when it starts
   * running. You can enable such reading by uncommenting the
   * following piece of code:
   *
     FullParamName = ros::this_node::getName()+"/run_period";
      
     if (true == Handle.getParam(FullParamName, RunPeriod))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
   */


  // Subscriber = Handle.subscribe("name_of_the_topic", length_of_the_queue, &ROSnode::MessageCallback, this);
  /***********************************************************
   * Uncomment if your node subscribes to the topic called
   * name_of_the_topic. Substitute length_of_the_queue with an
   * integer defining how many messages the input queue should
   * be able to contain.
   * NOTE: if your node is run periodically, make sure that the
   * queue is long enough to avoid that useful, but older, 
   * messages are discarded by ROS because the queue is full.
   * 
   * One such instruction is required for each topic that the node
   * subscribes to.
   */
  
  // Publisher = Handle.advertise<msg_pkg::msg_type>("name_of_the_topic", length_of_the_queue);
  /***********************************************************
   * Uncomment if your node publishes to the topic called
   * name_of_the_topic. Substitute length_of_the_queue with an
   * integer defining how many messages the output queue should
   * be able to contain.
   * msg_type must be substituted with the name of the type of 
   * message that your node will publish on the topic; msg_pkg 
   * must be substituted with the name of the ROS package which
   * contains the .msg file defining such message type.
   * A package which defines many common ROS message types is
   * std_msgs: a list of the message types it provides is here:
   * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
   *
   * One such instruction is required for each topic that the node
   * publishes to.
   */
   
  // Client = Handle.serviceClient<name_of_server_package::name_of_the_srv_file>("name_of_the_service");
  /***********************************************************
   * Uncomment if your node needs to act as a client of a service
   * called name_of_the_service, provided by a ROS server defined
   * by file name_of_the_srv_file.srv which is part of the ROS
   * package called name_of_the_server_package.
   *
   * Add one similar statement for each additional ROS server
   * that your node needs to access as a client.
   */
   
  // Service = Handle.advertiseService("name_of_the_service", &ROSnode::ProvideService, this);
  //ROS_INFO("ROS service %s available (provided by node %s).", "name_of_the_service", ros::this_node::getName().c_str());
  /***********************************************************
   * Uncomment the first statement if your node acts as a ROS
   * server, providing a service called name_of_the_service
   * to clients. The service is implemented by method
   * ProvideService.
   * Also uncomment the second statement if you want to highlight
   * the availability of the service, for instance for debugging
   * purposes.
   *
   * Add similar statements for each additional ROS service that
   * your node provides.
   */
   
  // TimeoutTimer = Handle.createTimer(ros::Duration(duration_of_the_timeout), &ROSnode::TimeoutCallback, this, put_here_true_or_false);
  /***********************************************************
   * Uncomment if your node requires a timeout. Substitute 
   * duration_of_the_timeout with the required value, in seconds,
   * expressed as a float (e.g., 14.0).
   * put_here_true_or_false should be substituted with true or
   * false. In the first case, after the timeout expires method
   * TimeoutCallback will be called only once ("once-only"
   * timeout). In the second case, once the timeout expires,
   * TimeoutCallback is called periodically, with period equal to
   * duration_of_the_timeout.
   * This statement also starts the timeout.
   *
   * You need one such instruction for each TimeoutTimer
   * attribute that you defined above.
   */
  
  /***********************************************************
   * RETRIEVING PARAMETER VALUES FROM THE ROS PARAMETER SERVER
   *
   * Uncomment the following piece of code if you want to
   * retrieve a parameter named my_param from the ROS parameter
   * server, and store its value in variable ParamVar (which
   * you need to have declared as a member variable of class
   * ROSNode; you should choose whether to make ParamVar a public
   * or private variable depending on who needs to access it).
   *
     // FullParamName = ros::this_node::getName()+"/my_param";
     //uncomment this if my_param is a private parameter of the
     //node, i.e. if its full name is 
     // /name_of_the_namespace/NAME_OF_THIS_NODE/my_param
     
     // FullParamName = Handle.getNamespace()+"my_param";
     //uncomment this if, instead, my_param is a global parameter
     //of the namespace that the node belongs to, i.e. if its
     //full name is /name_of_the_namespace/my_param
 
     if (true == Handle.getParam(FullParamName, ParamVar))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
   *
   * You need one piece of code like this for each parameter
   * value that your node needs to retrieve from the ROS parameter
   * server.
   */
   
   ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunContinuously(void)
{
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
  
  /* From ROS documentation:
   * "ros::spin() will not return until the node has been 
   * shutdown, either through a call to ros::shutdown() or a 
   * Ctrl-C." */
}


void ROSnode::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  while (ros::ok())
  {
    //PeriodicTask();
    /*********************************************************
     * Uncomment if you have defined a periodic task that the node
     * has to execute every time it is run. If you leave this
     * commented out, every time the node is run it only processes
     * callbacks.
     */
     
    ros::spinOnce();
    /* From ROS documentation:
     * "ros::spinOnce() will call all the callbacks waiting to be
     * called at that point in time. ." */
    
    LoopRate.sleep();
  }
}


void ROSnode::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
  /* 
   * ROS does not require you to do anything to shut down cleanly
   * your node. However, if your node needs to do do something
   * before shutting down, put the relevant code here.
   */
}


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::MessageCallback(const msg_pkg::msg_type::ConstPtr& msg)
{
  StopTimeout();
  The preceding statement blocks the timeout every time the 
  message callback is executed. Remove it if you don't want that
  to happen, or if your node does not use a timeout. 
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StopTimeout method that
  needs to be called.
  
  ...put here your code for the management of the message
  
  StartTimeout();
  The preceding statement restarts the timeout when the execution 
  of the message callback ends. Delete it if you don't want that
  to happen, or if your node does not use a timeout.
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StartTimeout method that
  needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::TimeoutCallback(const ros::TimerEvent& event)
{
  StopTimeout();
  Delete the preceding statement if your node does not use a
  timeout, or add a similar statement for each additional
  StopTimeout method that needs to be called.
  
  ...put here your code for the management of the timeout event
  
  StartTimeout();
  Delete the preceding statement if your node does not use a
  timeout, or add a similar statement for each additional
  StartTimeout method that needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::PeriodicTask(void)
{
  StopTimeout();
  The preceding statement blocks the timeout every time the 
  periodic task is executed. Delete it if you don't want that to
  happen, or if your node does not use a timeout. 
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StopTimeout method that
  needs to be called.
  
  ...put here your code for the management of the periodic task
  
  StartTimeout();
  The preceding statement restarts the timeout when the execution 
  of the periodic task ends. Delete it if you don't want that to
  happen, or if your node does not use a timeout.
  If your node uses more than one timeout, you may need to add a
  similar statement for each additional StartTimeout method that
  needs to be called.
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::UseService(void)
{
  Server.request.name_of_input = expression of appropriate type;
  
  where name_of_input must be substituted with the correct name of
  the input field, as defined by the .srv file of the server.
  
  if (Client.call(Server))
  {
    variable of appropriate type = Server.response.name_of_output;
    
    where name_of_output must be substituted with the correct name
    of the output field, as defined by the .srv file of the server.
  }
  else
  {
    Put here the code to manage the "server not responding" 
    condition. 
    Please note that if the server node is active but is not
    running at the time of the request by the client because it
    runs periodically and is currently sleeping, this is NOT a
    "server not responding" condition. The request of the client
    is queued by ROS, and the client will get the response at
    the next run of the server.
  }
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
bool ROSnode::ProvideService(name_of_the_server_package::name_of_the_srv_file::Request  &Req,  name_of_the_server_package::name_of_the_srv_file::Response &Res)
{
  suppose that the .srv file defines two input fields called in1
  and in2, and two output fields called out1 and out2. Then you
  have to write something like:
    
  Res.out1 = expression using Req.in1 and Req.in2;
  Res.out2 = expression using Req.in1 and Req.in2;
}
 *
 */


/*************************************************************
 * Uncomment the following blocks if you defined the corresponding
 * methods of ROSnode.
 *
void ROSnode::StopTimeout(void)
{
  TimeoutTimer.stop();
  ...add here similar statements to stop other timers, if needed
}

void ROSnode::StartTimeout(void)
{
  TimeoutTimer.setPeriod(ros::Duration(duration_of_the_timeout));
  ...substitute duration_of_the_timeout with the required value, in
  seconds, expressed as a float (e.g., 14.0)  
  TimeoutTimer.start();
  
  ...add here similar statements to start other timers, if needed
}
 *
 */


//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  /* NOTE: the call to ros::init should be the FIRST statement of
   * the 'main' block. You get compilation errors if you use any
   * part of the ROS system before that statement. */

  /* NOTE. If this node is launched with rosrun, a new node
   * gets added to the running ROS system, the name of which is
   * the string assigned to NAME_OF_THIS_NODE. If the node is
   * launched with roslaunch, it is possible that this choice of
   * name has been overridden, and that the newly added node takes
   * different name than NAME_OF_THIS_NODE. This happens when the
   * <node> statement in the launchfile used to launch the node
   * specifies a name for the node (which is not mandatory). */
  
  ROSnode MyNode;
   
  MyNode.Prepare();
  
  // MyNode.RunContinuously();
  // MyNode.RunPeriodically(MyNode.RunPeriod);
  /*
   * Uncomment ONE AND ONLY ONE of the above statements.
   */ 
   
  MyNode.Shutdown();
  
  return (0);
}


/*************************************************************
 * NOTE. If you want this file to be compiled and the
 * corresponding executable to be generated, remember to add a
 * suitable rosbuild_add_executable line to the CMakeLists.txt
 * file of your package. [Warning: the contents of this note is
 * valid for versions of ROS that use the (older) rosbuild
 * build system. They may be obsolete if your version of ROS is,
 * instead, based on the newer catkin build system.]
 *************************************************************/
