import { Box, Typography } from "@mui/material";
import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import "../App.css";

const Workers = () => {
  const [connected, setConnected] = useState(false);
  const [awsConnected, setAwsConnected] = useState(false);
  const [loggerConnected, setLoggerConnected] = useState(false);
  const [ArmConnected, setArmConnected] = useState(false);
  const [ros, setRos] = useState(null);
  const [messages, setMessages] = useState([]);
  const messagesEndRef = useRef(null);
  const messageSetRef = useRef(new Set()); 

  useEffect(() => {

    const rosInstance = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });

    rosInstance.on("connection", () => {
      console.log("Connection established!");
      setConnected(true);
      setRos(rosInstance);
      subscribeToWebsiteStatus(rosInstance);
      subscribeToAwsWebsiteStatus(rosInstance);
      subscribeToLoggerStatus(rosInstance);
      subscribeArmStatus(rosInstance)
    });

    rosInstance.on("close", () => {
      console.log("Connection is closed!");
      setConnected(false);

      setTimeout(() => {
        try {
          rosInstance.connect("ws://localhost:9090");
        } catch (error) {
          console.log("Connection problem");
        }
      }, 3000);
    });

    return () => {
      if (rosInstance) {
        rosInstance.close();
      }
    };
  }, []); 

  const subscribeArmStatus = (rosInstance: any) => {
    if (rosInstance) {
      try {
        const loggerListener = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/arm_status",
          messageType: "std_msgs/Bool",
        });
        loggerListener.subscribe((msg: any) => {
          console.log("Received message from /arm_status topic:", msg);
          setArmConnected(msg.data);
        });
  
        console.log("Subscription to /arm_status topic successful.");
      } catch (error) {
        console.error("Error during subscription to /arm_status topic:", error);
      }
    } else {
      console.error("ROS connection not initialized.");
    }
  };

  const subscribeToLoggerStatus = (rosInstance: any) => {
    if (rosInstance) {
      try {
        const loggerListener = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/logger_status",
          messageType: "std_msgs/Bool",
        });
        loggerListener.subscribe((msg: any) => {
          console.log("Received message from /logger_status topic:", msg);
          setLoggerConnected(msg.data);
        });
  
        console.log("Subscription to /logger_status topic successful.");
      } catch (error) {
        console.error("Error during subscription to /logger_status topic:", error);
      }
    } else {
      console.error("ROS connection not initialized.");
    }
  };

  const subscribeToAwsWebsiteStatus = (rosInstance: any) => {
    if (rosInstance) {
      try {
        const awsListener = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/aws_sender_status",
          messageType: "std_msgs/Bool",
        });
        awsListener.subscribe((msg: any) => {
          console.log("Received message from /aws_sender_status topic:", msg);
            setAwsConnected(msg.data);
  
        });

        console.log("Subscription to /aws_sender_status topic successful.");
      } catch (error) {
        console.error("Error during subscription to /aws_sender_status topic:", error);
      }
    } else {
      console.error("ROS connection not initialized.");
    }
  };

  const subscribeToWebsiteStatus = (rosInstance: any) => {
    if (rosInstance) {
      try {
        const statusListener = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/website_status",
          messageType: "std_msgs/String",
        });
//@ts-ignore
        statusListener.subscribe((message) => {
          console.log("Received message from /website_status topic:", message);

          const messageString = `${message.data}`;
          if (!messageSetRef.current.has(messageString)) {
            messageSetRef.current.add(messageString);
            //@ts-ignore 
            setMessages((prevMessages) => [
              ...prevMessages,
              { timestamp: new Date().toLocaleTimeString(), status: message.data },
            ]);
          }
        });

        console.log("Subscription to /website_status topic successful.");
      } catch (error) {
        console.error("Error during subscription to /website_status topic:", error);
      }
    } else {
      console.error("ROS connection not initialized.");
    }
  };

  useEffect(() => {

    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [messages]);

  return (
    <Box
      position={"absolute"}
      zIndex={9999999}
      top={"28%"}
      left={"80%"}
      display={"flex"}
      flexDirection="column"
      padding={"20px"}
      borderRadius={"8px"}
      width={"fit-content"}
      sx={{
        backgroundColor: "transparent",
      }}
    >
      <Box display={"flex"} flexDirection={"column"}>
        <Typography className="coding-font" fontSize={"25px"} fontWeight={600} color={"white"}>
          Website Status
          <span> 

          <div style={{
            display: "flex",
            flexDirection: "column",
            gap: "10px",
            fontSize: "14px"
          }}>
            <div style={{
              display: "flex",
              alignItems: "center",
              gap: "10px"
            }}>
              <span>ROS Connection:</span>
              <div style={{
                width: "25px",
                height: "25px",
                backgroundColor: connected ? "green" : "red",
                borderRadius: "10000px"
              }}></div>
            </div>
            <div style={{
              display: "flex",
              alignItems: "center",
              gap: "10px"
            }}>
              <span>AWS Connection:</span>
              <div style={{
                width: "25px",
                height: "25px",
                backgroundColor: awsConnected ? "green" : "red",
                borderRadius: "10000px"
              }}></div>
            </div>
            <div style={{
              display: "flex",
              alignItems: "center",
              gap: "10px"
            }}>
              <span>Logger Connection:</span>
              <div style={{
                width: "25px",
                height: "25px",
                backgroundColor: loggerConnected ? "green" : "red",
                borderRadius: "10000px"
              }}></div>
            </div>
            <div style={{
              display: "flex",
              alignItems: "center",
              gap: "10px"
            }}>
              <span>Arm Status:</span>
              <div style={{
                width: "25px",
                height: "25px",
                backgroundColor: ArmConnected ? "green" : "red",
                borderRadius: "10000px"
              }}></div>
            </div>
          </div>

          </span>

        </Typography>
        
        <Box
          sx={{
            height: "200px",
            overflowY: "auto",
            display: "flex",
            flexDirection: "column",
            paddingRight: "10px", // To avoid content hiding behind scrollbar
          }}
        >
          {messages.length > 0 ? (
            <>
              {messages.map((msg, index) => (
                <Box key={index} mb={1}>
                  <Typography className="coding-font" fontSize={"12px"} fontWeight={500} color={"white"}>
                    
                    [{msg.timestamp}] {msg.status}
                  </Typography>
                </Box>
              ))}
              {/* This empty div is used to scroll to the bottom */}
              <div ref={messagesEndRef} />
            </>
          ) : (
            <Typography fontSize={"12px"} className="coding-font" fontWeight={500} color={"white"}>
              No status messages yet.
            </Typography>
          )}
        </Box>
      </Box>
    </Box>
  );
};

export default Workers;
