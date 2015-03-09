classdef ACCModes < Simulink.IntEnumType
  enumeration
    off(0)
    on(1)
    cruise(2)
    standby(3)   
   end
  methods (Static)
    function retVal = getDefaultValue()
      retVal = ACCModes.off;
    end
  end
end

