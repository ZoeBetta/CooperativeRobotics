function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % add policy for changing phase
            [ang_e, lin_e] = CartError(uvms.wTggv , uvms.wTv);
            if (norm(ang_e) < 0.05 && norm(lin_e) < 0.1)
                mission.phase = 2;
                mission.phase_time = 0;
                disp(' *** change to phase 2 *** ');
            end
            

            
        case 2
            if (norm(uvms.v_rho_o)<0.1 && uvms.a<0.05)
                mission.phase=3;
                mission.phase_time=0;
                disp(' *** change to phase 3 *** ');
                
            end
            
        case 3
            

    end
end

