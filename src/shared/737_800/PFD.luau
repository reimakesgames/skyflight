local ReplicatedStorage = game:GetService("ReplicatedStorage")
local fusion = require(ReplicatedStorage.Packages.fusion)

local New = fusion.New
local Children = fusion.Children

return function(aircraft)
	return New "Frame" {
		BackgroundColor3 = Color3.fromRGB(31, 31, 31),
		BorderSizePixel = 0,
		Size = UDim2.new(0, 250, 0, 250),

		[Children] = {
			New "Frame" {
				BackgroundColor3 = Color3.fromRGB(255, 255, 255),
				BorderSizePixel = 0,
				Position = UDim2.new(0.5, 0, 0.5, 0),
				Size = UDim2.new(0, 125, 0, 125),

				[Children] = {
					New "Frame" {
						BackgroundTransparency = 1,
						Position = UDim2.new(0.5, 0, 0.5, 0),

					} -- RPI anchor
				}
			}, -- Roll and Pitch indicator
		}
	}
end
