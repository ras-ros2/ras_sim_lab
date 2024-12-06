import React, { useEffect, useState } from "react";
import { Typography, Box } from "@mui/material";
import { shared } from "./shared";

interface JSONData {
  lab_name: string;
  setup_objects: {
    base: {
      id_1: {
        location: {
          [key: string]: string | number;
        };
      };
    };
  };
}
interface Props {
  open: boolean;
  exp_num: string;
}
const PopUp: React.FC<Props> = ({ open }) => {
  const [jsonData, setJsonData] = useState<JSONData | null>(null);
  const [r, setR] = useState("");
  const exp = shared.get("inputA");
  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(`http://localhost:8500/yaml/${exp}`);
        if (!response.ok) {
          throw new Error("Failed to fetch YAML data");
        }
        const data = await response.json();
        setJsonData(data);
        const re = JSON.stringify(data, null, 2);
        setR(re);
      } catch (error) {
        console.error("Error fetching YAML data:", error);
      }
    };

    if (open) {
      fetchData();
    }
  }, [open, exp]);

  return (
    <>
      <Box
        sx={{
          display: open ? "block" : "none",
          position: "absolute",
          top: "-30%",
          right: "-200%",
          // backgroundColor: "whitesmoke",
          borderRadius: "24px",
          padding: "25px",
          maxHeight: "500px",
          overflowY: "hidden",
        }}
      >
        {" "}
        <div>
          {jsonData && (
            <Typography color={"black"}>
              <pre>{r}</pre>
            </Typography>
          )}
        </div>
      </Box>
    </>
  );
};

export default PopUp;
