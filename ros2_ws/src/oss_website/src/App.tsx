// import { MuiTypography } from "./components/MuiTypography";
import "./App.css";
import "./index.css";
import videoBg from "./assets/videoBg.mp4";
import { MuiButton } from "./components/MuiButton";
import MultiTextfield from "./components/MultiTextfield";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import Status from "./components/status";

function App() {
  const theme = createTheme({
    direction: "rtl",
    typography: {
      fontSize: 11,
      fontWeightLight: 300,
      fontWeightRegular: 400,
      fontWeightMedium: 500,
    },
  });

  return (
    <ThemeProvider theme={theme}>
      <div>
       
        <ResponsiveAppBar />

        <video src={videoBg} autoPlay loop muted />

        <MuiButton />
        <Status/>
        <MultiTextfield />
      </div>
    </ThemeProvider>
  );
}

export default App;
