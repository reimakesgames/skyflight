local Debris = game:GetService("Debris")
local ASL_DENSITY = 1.225 -- kg/m^3
local GRAVITY = 9.81 -- m/s^2
local FEET_IN_STUDS = 0.82021

local PFD_ASI_KNOT_PIXELS = 1686 / 1069.5
local PFD_ATI_PITCH_PIXELS = 248 / 90
local PFD_ALI_THOU_FT_PIXELS = 14596 / 62
-- local PFD_ALI_BOTTOM_MARGIN = 143 + (PFD_ALI_THOU_FT_PIXELS * 2)
local PFD_ALI_BOTTOM_MARGIN = 520

local WING_SPAN = 34.3 -- m
local WING_AREA = 124.6 -- m^2
local WING_ASPECT_RATIO = 9.45
local WING_TAPER_RATIO = 0.16
local WING_ROOT_CHORD = 7.88 -- m
local WING_TIP_CHORD = 1.25 -- m
local WING_MEAN_AERODYNAMIC_CHORD = 3.96 -- m
local WING_DIHEDRAL_ANGLE = 6 -- degrees
local WING_ONE_FOURTH_CHORD_SWEEP_ANGLE = 25 -- degrees

local FLAP_SPAN = 0.6 -- m
local FLAP_AREA = 0.3 -- m^2

local FIN_SPAN = 7.16 -- m
local FIN_AREA = 26.44 -- m^2
local FIN_RUDDER_AREA = 5.22 -- m^2
local FIN_ASPECT_RATIO = 1.91
local FIN_TAPER_RATIO = 0.27
local FIN_SWEEP_ANGLE = 35 -- degrees

local HORIZONTAL_STABILIZER_SPAN = 14.35 -- m
local HORIZONTAL_STABILIZER_TAILPLANE_AREA = 32.78 -- m^2
local HORIZONTAL_STABILIZER_ELEVATORS_AREA = 6.55 -- m^2
local HORIZONTAL_STABILIZER_ASPECT_RATIO = 6.55
local HORIZONTAL_STABILIZER_TAPER_RATIO = 0.2
local HORIZONTAL_DIHEDRAL_ANGLE = 7
local HORIZONTAL_STABILIZER_ONE_FOURTH_CHORD_SWEEP_ANGLE = 30 -- degrees

local DRY_MASS = 41_140 -- kg
local MAX_TAKEOFF_MASS = 79_015 -- kg

local ENGINE_STATIC_THRUST = 107_000 -- N
local ENGINE_IDLE_THRUST = 0.2 * ENGINE_STATIC_THRUST
local THRUST_CONSTANT_INCREMENT = 20_000 -- N / s
local THRUST_CONSTANT_DECREMENT = 10_000 -- N / s

-- a function that returns a number with a specified zero count
-- eg 80, 4 becomes 0080, or 4, 2 becomes 04
local function zeroPad(number, zeros)
	local numberString = tostring(number)
	local numberLength = #numberString
	local zeroCount = zeros - numberLength
	if zeroCount > 0 then
		return ("0"):rep(zeroCount) .. numberString
	else
		return numberString
	end
end

local Aircraft = {}
Aircraft.__index = Aircraft

export type Aircraft = {
	Sensor: BasePart?,
	SensorOldPosition: Vector3?,
	AirSpeed: number?,
	GroundSpeed: number?,
	Direction: Vector3?,

	TargetThrustPercentage: number?,
	CurrentThrust: number?,
}

function Aircraft.new()
	local self = {}
	self.AircraftModel = nil

	self.Sensor = nil
	self.SensorOldPosition = nil
	self.AirSpeed = 0
	self.GroundSpeed = 0
	self.Direction = Vector3.new(0, 0, 0)
	self.Velocity = Vector3.new(0, 0, 0)

	self.TargetThrustPercentage = 0
	self.CurrentThrust = ENGINE_IDLE_THRUST

	self.TargetRoll = 0
	self.TargetPitch = 0

	self.SimulationSpeed = 1

	return setmetatable(self, Aircraft)
end

local liftPower = {
	{90, 0},
	{60, 0},
	{20, 0.8},
	{0, 0.5},
	{-3, 0},
	{-10, -0.3},
	{-20, -0.5},
	{-60, 0},
	{-90, 0},
}

local function lerp(a, b, t)
    return a + (b - a) * t
end

local function CalculateAOAValue(value)
    local lowerIndex = 1
    local upperIndex = 2

    -- Find the two values in the table that surround the given value
    for i = 1, #liftPower - 1 do
        if value <= liftPower[i][1] and value >= liftPower[i + 1][1] then
            lowerIndex = i
            upperIndex = i + 1
            break
        end
    end

    -- Interpolate the corresponding values
    local lowerValue = liftPower[lowerIndex][2]
    local upperValue = liftPower[upperIndex][2]

    local lowerInput = liftPower[lowerIndex][1]
    local upperInput = liftPower[upperIndex][1]

	-- print(lowerValue, upperValue, lowerInput, upperInput)

    local t = (value - lowerInput) / (upperInput - lowerInput)
    return lerp(lowerValue, upperValue, t)
end

function Aircraft:CalculateLift(wingArea, angleOfAttack)
	-- print(math.floor(angleOfAttack * 10) / 10, CalculateAOAValue(angleOfAttack))
	local CL = 0.5 * CalculateAOAValue(angleOfAttack)
	local Lift = CL * ((ASL_DENSITY * self.AirSpeed ^ 2) / 2) * wingArea * 1.2
	return Lift
end

--[[
	Sets the aircraft's model
]]
function Aircraft:SetAircraftModel(model)
	self.AircraftModel = model
	self.Direction = model.PrimaryPart.CFrame.LookVector
end

--[[
	Sets the aircraft's sensor, this is a very abstract sensor and is not a pitot, static, gyro, etc.
	This can be just a part that is used to determine the aircraft's orientation and ground speed.
]]
function Aircraft:SetSensor(sensor)
	self.Sensor = sensor
end

--[[
	Sets the aircraft's engine sound sources
]]
function Aircraft:SetEngineSoundSources(engine1, engine2)
	self.EngineSoundSource1 = engine1
	self.EngineSoundSource2 = engine2
end

--[[
	Sets the aircraft's PFD display
]]
function Aircraft:SetPFD(pfd)
	self.PFD = pfd
end

local counter = 1
local function NewPositionMarker(self)
	if counter > 10 then
		counter = 1
		local marker = Instance.new("Part")
		Debris:AddItem(marker, 20)
		marker.Anchored = true
		marker.CanCollide = false
		marker.CanTouch = false
		marker.CanQuery = false
		marker.Size = Vector3.new(8, 8, 8)
		marker.Transparency = 0.5
		marker.Color = Color3.fromRGB(0, 255, 0)
		marker.CFrame = self.AircraftModel.PrimaryPart.CFrame
		marker.Parent = workspace
	end
	counter = counter + 1
end

local PreviousVelocity = Vector3.new(0, 0, 0)
--[[
	A function that should be ran via Stepped
]]
function Aircraft:Update(dt)
	dt *= self.SimulationSpeed

	workspace.CurrentCamera.CameraSubject = self.AircraftModel.Camera
	NewPositionMarker(self)

	local pitch, yaw, roll = self.Sensor.CFrame:ToOrientation()

	local GravityVector = Vector3.new(0, -GRAVITY, 0)

	local ThrustLine1 = self.AircraftModel.ThrustLine1
	local ThrustLine2 = self.AircraftModel.ThrustLine2
	local CenterOfMass = self.AircraftModel.CenterOfMass

	local ThrustToReach = ((ENGINE_STATIC_THRUST - ENGINE_IDLE_THRUST) * self.TargetThrustPercentage) + ENGINE_IDLE_THRUST
	if self.CurrentThrust < ThrustToReach then
		self.CurrentThrust = math.min(self.CurrentThrust + (THRUST_CONSTANT_INCREMENT * dt), ThrustToReach)
	elseif self.CurrentThrust > ThrustToReach then
		self.CurrentThrust = math.max(self.CurrentThrust - (THRUST_CONSTANT_DECREMENT * dt), ThrustToReach)
	end

	local ThrustLine1Position = ThrustLine1.Position
	local ThrustLine2Position = ThrustLine2.Position
	local CenterOfMassPosition = CenterOfMass.Position
	local ThrustLine1ToCenterOfMass = (ThrustLine1Position - CenterOfMassPosition).Magnitude / 4
	local ThrustLine2ToCenterOfMass = (ThrustLine2Position - CenterOfMassPosition).Magnitude / 4
	local ThrustLine1Moment = ThrustLine1ToCenterOfMass * self.CurrentThrust
	local ThrustLine2Moment = ThrustLine2ToCenterOfMass * self.CurrentThrust
	local TotalMoment = ThrustLine1Moment + ThrustLine2Moment
	local PitchUp = TotalMoment / (DRY_MASS * GRAVITY^2)

	self.AircraftModel:SetPrimaryPartCFrame(self.AircraftModel.PrimaryPart.CFrame * CFrame.Angles(math.rad(PitchUp) * dt, 0, 0))

	local verticalVelocity = Vector2.new(self.Velocity.Y, self.Velocity.Z)
	local verticalAircraftDirection = Vector2.new(self.AircraftModel.CenterOfMass.CFrame.LookVector.Y, self.AircraftModel.CenterOfMass.CFrame.LookVector.Z)
	local verticalAngleOfAttack = math.deg(math.atan2(verticalVelocity.Y, verticalVelocity.X) - math.atan2(verticalAircraftDirection.Y, verticalAircraftDirection.X))

	local Lift = self:CalculateLift(WING_AREA, verticalAngleOfAttack)
	local LiftForce = self.AircraftModel.PrimaryPart.CFrame.UpVector * (Lift / DRY_MASS)
	local ThrustForce = self.Direction * ((self.CurrentThrust * 2) / DRY_MASS)
	local Acceleration = GravityVector + ThrustForce + LiftForce

	-- self.AircraftModel:SetPrimaryPartCFrame(self.AircraftModel.PrimaryPart.CFrame * CFrame.Angles(math.rad(Lift / 100) * dt, 0, 0))

	-- find the floor by raycasting 10 studs below COM
	local Params = RaycastParams.new()
	Params.FilterType = Enum.RaycastFilterType.Exclude
	Params.FilterDescendantsInstances = {self.AircraftModel}
	local Floor = workspace:Raycast(CenterOfMass.Position, Vector3.new(0, -10.5, 0), Params)
	if Floor then
		self.Velocity = Vector3.new(self.Velocity.X, 0, self.Velocity.Z)
		-- move the aircraft up to the floor
		local NewPosition = Floor.Position + Vector3.new(0, 10.49, 0)
		Acceleration = Vector3.new(Acceleration.X, 0, Acceleration.Z)
		self.AircraftModel:SetPrimaryPartCFrame(CFrame.new(NewPosition, NewPosition + self.Direction))
	end

	-- roll the aircraft
	local RollToReach = self.TargetRoll
	local AppliedRoll = 0
	if roll < RollToReach - 0.2 then
		AppliedRoll = math.rad(10) * (self.AirSpeed / 150)
	elseif roll > RollToReach + 0.2 then
		AppliedRoll = math.rad(-10) * (self.AirSpeed / 150)
	end
	if Floor then
		AppliedRoll = 0
	end

	-- pitch the aircraft
	local PitchToReach = self.TargetPitch
	local AppliedPitch = 0
	if pitch < PitchToReach - 0.2 then
		AppliedPitch = math.rad(10) * (self.AirSpeed / 150)
	elseif pitch > PitchToReach + 0.2 then
		AppliedPitch = math.rad(-10) * (self.AirSpeed / 150)
	end

	self.AircraftModel:SetPrimaryPartCFrame(self.AircraftModel.PrimaryPart.CFrame * CFrame.Angles(AppliedPitch * dt, 0, AppliedRoll * dt))

	self.Velocity = self.Velocity + (Acceleration * dt)
	local ProgradeSpeed = self.Velocity:Dot(self.Direction)
	self:SetSpeed(ProgradeSpeed)

	self.AircraftModel:SetPrimaryPartCFrame(self.AircraftModel.PrimaryPart.CFrame + (self.Velocity * 4 * dt))
	self.AircraftModel:SetPrimaryPartCFrame(self.AircraftModel.PrimaryPart.CFrame:Lerp(CFrame.new(self.AircraftModel.PrimaryPart.Position, self.AircraftModel.PrimaryPart.Position + self.Velocity), (self.AirSpeed / 400) * dt))

	self.EngineSoundSource1.Lead.PlaybackSpeed = 0.5 + (((self.CurrentThrust - ENGINE_IDLE_THRUST) / (ENGINE_STATIC_THRUST - ENGINE_IDLE_THRUST)) * 1.5)
	self.EngineSoundSource2.Lead.PlaybackSpeed = 0.5 + (((self.CurrentThrust - ENGINE_IDLE_THRUST) / (ENGINE_STATIC_THRUST - ENGINE_IDLE_THRUST)) * 1.5)

	self.Direction = self.AircraftModel.PrimaryPart.CFrame.LookVector

	if not self.Sensor then
		self.SensorOldPosition = nil
	else
		local speed = self.AirSpeed
		if self.PFD then
			local Ladder = self.PFD.AttitudeDisplay.Clipper.Ladder.Ladder
			local Background = self.PFD.AttitudeDisplay.Clipper.Background
			local IndicatedAirspeedBar = self.PFD.IndicatedAirspeedBar
			local IndicatedAirspeed = self.PFD.IndicatedAirspeed
			local IndicatedAltitudeBar = self.PFD.IndicatedAltitudeBar
			local IndicatedAltitude = self.PFD.IndicatedAltitude

			local pitchPixels = math.deg(pitch) * PFD_ATI_PITCH_PIXELS
			local offset = UDim2.fromOffset(pitchPixels * -math.sin(roll), pitchPixels * math.cos(roll))

			Ladder.Rotation = math.deg(roll)
			Ladder.Position = UDim2.new(0.5, 0, 0.5, 0) + offset

			Background.Rotation = math.deg(roll)
			Background.Position = UDim2.new(0.5, 0, 0.5, 0) + offset

			IndicatedAirspeedBar.Center.Position = UDim2.new(0, 0, 1, 2) + UDim2.fromOffset(0, speed * PFD_ASI_KNOT_PIXELS)
			IndicatedAirspeed.LabelArea.Label.Text = zeroPad(math.floor(speed), 3)

			IndicatedAltitudeBar.Center.Position = UDim2.new(0, 0, 1, PFD_ALI_BOTTOM_MARGIN) + UDim2.fromOffset(0, ((self.AircraftModel.PrimaryPart.Position.Y / 1000) * FEET_IN_STUDS) * PFD_ALI_THOU_FT_PIXELS)
			IndicatedAltitude.LabelArea.FL.Text = zeroPad(math.floor((self.AircraftModel.PrimaryPart.Position.Y * FEET_IN_STUDS) / 1000), 2)
			IndicatedAltitude.LabelArea.FT.Text = zeroPad(math.floor((self.AircraftModel.PrimaryPart.Position.Y * FEET_IN_STUDS) % 1000), 3)

			-- calculate vertical G forces
			local difference = self.Velocity.Y - PreviousVelocity.Y
			local verticalGForce = difference / dt / GRAVITY
			PreviousVelocity = self.Velocity
			self.PFD.VerticalGForce.Text = `{math.floor(verticalGForce * 10) / 10}G`
			self.PFD.SimulationSpeed.Text = `{self.SimulationSpeed}x`

			self.PFD.TextLabel.Text = `{math.round(math.deg(yaw)+180)}°`
		end
	end
end

--[[
	Updates the aircraft's groundSpeed in knots
]]
function Aircraft:SetSpeed(speed)
	self.GroundSpeed = speed
	self.AirSpeed = self.GroundSpeed
end

--[[
	Updates the aircraft's engines' thrust by a percentage
]]
function Aircraft:SetThrust(thrustPercentage)
	self.TargetThrustPercentage = thrustPercentage
end

--[[
	Updates the aircraft's roll by degrees
]]
function Aircraft:SetRoll(roll)
	self.TargetRoll = -roll
end

function Aircraft:SetPitch(pitch)
	self.TargetPitch += pitch
end

function Aircraft:SetSimulationSpeed(speed)
	self.SimulationSpeed *= speed
end

return Aircraft
