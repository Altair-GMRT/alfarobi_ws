import React from "react";
import AppBar from "../components/AppBar";
import Dropdown from "../components/Dropdown";
import InputCard from "../components/InputCard";
import robotBody from "../assets/robot_body_joint.png";
import { useParams } from "react-router-dom";

function InitialPoses() {
  const { robot } = useParams();
  return (
    <div className="flex flex-col h-screen bg-secondary_bg">
      <AppBar />
      <div className="flex flex-row">
        <div className="flex flex-row align-center mt-4 mb-12 mx-4 p-2 px-4 bg-primary_bg rounded-xl">
          <p className="text-white text-1xl">Body module : </p>
          <Dropdown color={"white"} />
        </div>
      </div>
      <button
        onClick={() => {
          console.log("clicked");
        }}
      >
        save
      </button>
      <div className="flex flex-row w-screen justify-center">
        <div className="flex flex-col w-[31vw]">
          <InputCard robot={robot} titles={["r_sho_roll", "r_sho_pitch"]} />
          <InputCard robot={robot} titles={["r_el_pitch"]} />
          <InputCard
            robot={robot}
            titles={["r_hip_roll", "r_hip_pitch", "r_hip_yaw"]}
          />
          <InputCard robot={robot} titles={["r_knee_pitch"]} />
          <InputCard robot={robot} titles={["r_ank_roll", "r_ank_pitch"]} />
        </div>
        <div className="grid justify-center items-center">
          <img src={robotBody} alt="robot" className="w-[32vw] h-[74vh]" />
        </div>
        <div className="flex flex-col w-[31vw] text-center space-x-2">
          <div className="flex flex-col w-[31vw]">
            <InputCard robot={robot} titles={["head_tilt", "head_pan"]} />
            <InputCard robot={robot} titles={["l_sho_roll", "l_sho_pitch"]} />
            <InputCard robot={robot} titles={["l_el_pitch"]} />
            <InputCard
              robot={robot}
              titles={["l_hip_roll", "l_hip_pitch", "l_hip_yaw"]}
            />
            <InputCard robot={robot} titles={["l_knee_pitch"]} />
            <InputCard robot={robot} titles={["l_ank_roll", "l_ank_pitch"]} />
          </div>
        </div>
      </div>
    </div>
  );
}

export default InitialPoses;
