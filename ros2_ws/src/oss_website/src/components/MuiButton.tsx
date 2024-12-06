import { Button, Stack } from "@mui/material";
import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";
import Message from "../message.tsx";
import { shared } from "./shared.ts";
import PopUp from "./PopUp.tsx";

export const MuiButton = () => {
  const [ros, setRos] = useState(null);
  useEffect(() => {}, []);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });

    ros.on("connection", () => {
      console.log("Connected to ROS");
    });

    ros.on("close", () => {
      console.log("Disconnected from ROS");
    });

    setRos(ros);

    return () => {
      ros.close();
    };
  }, []);

  const RealClick = () => {
    var botClient = new ROSLIB.Action({
      ros: ros,
      name: "/execute_exp",
      actionType: "oss_interfaces/ExecuteExp",
    });

    // Send an action goal
    var goal = new ROSLIB.ActionGoal({});

    var goal_id = botClient.sendGoal(
      goal,
      function (result) {
        console.log(
          "Result for action goal on " +
            botClient.name +
            ": " +
            result.result.success
        );
      },
      function (feedback) {
        console.log(
          "Feedback for action on " +
            botClient.name +
            ": " +
            feedback.picked_object
        );
      }
    );
  };
  const [open, setOpen] = useState(false);
  const LoadClick = () => {
    const value1 = shared.get("inputA");
    const value2 = shared.get("inputB");
    setOpen(!open);
    var simClient = new ROSLIB.Service({
      ros: ros,
      name: "/get_exepriment",
      serviceType: "oss_interfaces/LoadExp.srv",
    });
    var request = new ROSLIB.ServiceRequest({
      exepriment_id: value1,
      instruction_no: value2,
    });

    simClient.callService(request, function (result) {
      console.log(
        "Result for service call on " + simClient.name + ": " + result.success
      );
    });
  };

  const SyncClick = () => {
    var SyncClient = new ROSLIB.Service({
      ros: ros,
      name: "/reset_arm",
      serviceType: "std_srvs/SetBool.srv",
    });
    var request = new ROSLIB.ServiceRequest({});

    SyncClient.callService(request, function (result) {
      console.log(
        "Result for service call on " + SyncClient.name + ": " + result.success
      );
    });
  };

  const SimTestClick = () => {
    var simTestClient = new ROSLIB.Service({
      ros: ros,
      name: "/test_experiment",
      serviceType: "std_srvs/SetBool.srv",
    });
    var request = new ROSLIB.ServiceRequest({});

    simTestClient.callService(request, function (result) {
      console.log(
        "Result for service call on " +
          simTestClient.name +
          ": " +
          result.success
      );
    });
  };

  const CancelClick = () => {
    var cancel = new ROSLIB.Service({
      ros : ros,
      name : '/hard_reset',
      serviceType : 'std_srvs/Empty.srv'
    });
    var request = new ROSLIB.ServiceRequest({});

    cancel.callService(request, function (result) {
      console.log(
        "Result for service call on " +
        cancel.name +
          ": " +
          result.success
      );
    });
  }

  return (
    <div
      className="button"
      style={{
        position: "absolute",
        top: "47%",
        left: "20%",
        transform: "translate(-50%, -50%)",
      }}
    >
      <PopUp open={open} />
      <Stack direction="column" spacing={2}>
        <Stack direction="row" spacing={2}>
          <Button
            variant="contained"
            style={{ backgroundColor: "purple" }}
            onClick={LoadClick}
          >
            Load Experiment
          </Button>
          <Button
            variant="contained"
            style={{ backgroundColor: "purple" }}
            onClick={RealClick}
          >
            Start Real Arm
          </Button>
        </Stack>
        <Button variant="contained" style={{ backgroundColor: "Red" }} onClick={CancelClick}>
          Hard Reset
        </Button>
        <Button
          variant="contained"
          style={{ backgroundColor: "purple" }}
          onClick={SimTestClick}
        >
          Start Simulation
        </Button>
        <Button
          variant="contained"
          style={{ backgroundColor: "purple" }}
          onClick={SyncClick}
        >
          Reset Arm
        </Button>
      </Stack>

      <script
        type="text/javascript"
        src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"
      ></script>
    </div>
  );
};
