local Players = game:GetService("Players")
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")
local UserInputService = game:GetService("UserInputService")
local AircraftClass = require(ReplicatedStorage.Shared.Aircraft)

local LocalPlayer = Players.LocalPlayer
local PlayerGui = LocalPlayer:WaitForChild("PlayerGui")

local ScreenGui = PlayerGui:WaitForChild("ScreenGui")
local PFD = ScreenGui:WaitForChild("PFD")

local Aircraft = AircraftClass.new()
local AircraftModel = workspace.Aircraft
local Sensor = AircraftModel.Sensor

Aircraft:SetAircraftModel(AircraftModel)
Aircraft:SetSensor(Sensor)
Aircraft:SetEngineSoundSources(AircraftModel.MainParts.Sound1, AircraftModel.MainParts.Sound2)
Aircraft:SetPFD(PFD)
RunService.Stepped:Connect(function(t, dt)
	Aircraft:Update(dt)
end)

UserInputService.InputBegan:Connect(function(input, gameProcessed)
	if gameProcessed then return end
	if input.KeyCode == Enum.KeyCode.W then
		Aircraft:SetThrust(1)
	elseif input.KeyCode == Enum.KeyCode.S then
		Aircraft:SetThrust(0)
	elseif input.KeyCode == Enum.KeyCode.Q then
		Aircraft:SetRoll(-30)
	elseif input.KeyCode == Enum.KeyCode.E then
		Aircraft:SetRoll(30)
	elseif input.KeyCode == Enum.KeyCode.R then
		Aircraft:SetPitch(1)
	elseif input.KeyCode == Enum.KeyCode.F then
		Aircraft:SetPitch(-1)
	elseif input.KeyCode == Enum.KeyCode.Z then
		Aircraft:SetSimulationSpeed(0.5)
	elseif input.KeyCode == Enum.KeyCode.X then
		Aircraft:SetSimulationSpeed(2)
	end
end)

UserInputService.InputEnded:Connect(function(input, gameProcessed)
	if gameProcessed then return end
	if input.KeyCode == Enum.KeyCode.Q then
		Aircraft:SetRoll(0)
	elseif input.KeyCode == Enum.KeyCode.E then
		Aircraft:SetRoll(0)
	end
end)
