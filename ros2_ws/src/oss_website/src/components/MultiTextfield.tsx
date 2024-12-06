/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable @typescript-eslint/ban-types */
import { TextField, Stack } from "@mui/material";
import * as React from "react";
import { shared } from "./shared.ts";
export interface State {
  [key: string]: State[keyof State];
  inputA: string;
  inputB: string;
}
export default class App extends React.Component<{}, State> {
  constructor(props: {}) {
    super(props);
    this.state = {
      inputA: "",
      inputB: "",
    };
  }
  handleChange = (field: string) => (event: any) => {
    const value = event.target.value.toUpperCase();
    this.setState({ [field]: value });
  };

  render() {
    // console.log(this.state);
    shared.set("inputA", this.state["inputA"]);
    shared.set("inputB", this.state["inputB"]);

    return (
      <div
        className="center-container"
        style={{
          position: "absolute",
          top: "30%",
          left: "20%",
          transform: "translate(-50%, -50%)",
        }}
      >
        <Stack direction="column" spacing={4}>
          <TextField
            className="text"
            label="EXPERIMENT NO"
            onChange={this.handleChange("inputA")}
            value={this.state["inputA"]}
            sx={{
              width: 200,
              height: 40,
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".3rem",
              color: "White",
              textDecoration: "none",
            }}
            style={{ backgroundColor: "rgba(255, 255, 255, 0.8)" }}
            size="small"
            margin="normal"
            color="secondary"
          />
          <TextField
            className="text"
            label="INSTRUCTION ID"
            onChange={this.handleChange("inputB")}
            value={this.state["inputB"]}
            sx={{
              width: 200,
              height: 40,
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".3rem",
              color: "White",
              textDecoration: "none",
            }}
            style={{ backgroundColor: "rgba(255, 255, 255, 0.8)" }}
            size="small"
            margin="normal"
            color="secondary"
          />
        </Stack>
      </div>
    );
  }
}
