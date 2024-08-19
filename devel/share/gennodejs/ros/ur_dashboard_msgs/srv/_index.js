
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let GetProgramState = require('./GetProgramState.js')
let Load = require('./Load.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let RawRequest = require('./RawRequest.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetRobotMode = require('./GetRobotMode.js')
let AddToLog = require('./AddToLog.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  GetProgramState: GetProgramState,
  Load: Load,
  GetLoadedProgram: GetLoadedProgram,
  GetSafetyMode: GetSafetyMode,
  IsInRemoteControl: IsInRemoteControl,
  RawRequest: RawRequest,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  GetRobotMode: GetRobotMode,
  AddToLog: AddToLog,
};
