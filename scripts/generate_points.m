function [rotations, positions] = generate_points(do_plot, write_yaml)
  % This matlab script generates poses for synthetic view generation of an object

  % Zenith angles
  zenith_min = 20;
  zenith_max = 60;
  zenith_step = 10;
  zenith_angles = pi/180*linspace(20,60,6);%(zenith_min:zenith_step:zenith_max);

  % Azimuth angles
  azimuth_min = 0;
  azimuth_max = 360;
  azimuth_step = 15;
  azimuth_angles = pi/180*linspace(0,360,36);%,(azimuth_min:azimuth_step:azimuth_max);
  azimuth_angles = azimuth_angles(1:end-1)

  % Camera axis rotation
  camera_angles = pi/180*linspace(0,360,15);

  % Shell distances
  radii = [0.14, 0.15, 0.16, 0.18, 0.20];

  % Compute number of poses
  n_poses = length(radii) * length(azimuth_angles) * length(zenith_angles) * length(camera_angles);
  disp(sprintf('Generating %d poses...',n_poses))

  % Pre-allocate pose matrices
  positions = zeros(n_poses,3);
  rotations = zeros(n_poses,9);
  index = 1;

  for r = radii

    for za = zenith_angles
      R_za = Ry(za);

      for aa = azimuth_angles
        R_aa = Rz(aa);
        R_orbit = R_aa*R_za;
        
        pos = [r 0 0]';
        pos = R_orbit*pos;

        for ca = camera_angles
          % [r, za, aa, ca]
          positions(index,:) = pos;

          % Compute the rotation from basis columns
          % Normalize vector pointing at origin
          rotz = R_orbit*[-1 0 0]'; 
          roty = R_orbit*[0 0 1]';
          rotx = cross(roty,rotz);

          rot = [rotx roty rotz]';
          rot = Rz(ca)'*rot;

          rotations(index,:) = reshape(rot',1,[]);
          
          % Increment index
          index = index + 1;
        end
      end
    end
  end

  if do_plot
    figure(1);
    clf;
    plot3(positions(:,1),positions(:,2), positions(:,3),'kx')
    axis equal
    grid on
    hold on
    for index = 1:length(camera_angles):length(positions)
      plot_triad(0.02, positions(index,:)', reshape(rotations(index,:)',3,3))
    end
  end

  % Write out values to yaml (row-major matrices)
  if write_yaml
    yaml_str = '%YAML:1.0';
    yaml_str = [yaml_str,sprintf('\npose_type: camera')];
    yaml_str = [yaml_str,sprintf('\nn_poses: %d', length(positions))];
    yaml_str = [yaml_str,sprintf('\nposes:')];
    for index = 1:length(positions)
      new_row = sprintf('\n  - { p: [%f, %f, %f], R: [%f, %f, %f, %f, %f, %f, %f, %f, %f] }', ...
                        positions(index,1), ...
                        positions(index,2), ...
                        positions(index,3), ...
                        rotations(index,1), rotations(index,2), rotations(index,3), ...
                        rotations(index,4), rotations(index,5), rotations(index,6), ...
                        rotations(index,7), rotations(index,8), rotations(index,9)); 
      yaml_str = [yaml_str,new_row];
    end

    f = fopen('poses.yml','w');
    fwrite(f, yaml_str);
    fclose(f);

  end

end

function plot_triad(s,p,R)
  x = p+R*[s;0;0];
  y = p+R*[0;s;0];
  z = p+R*[0;0;s];
  plot3([p(1) x(1)], [p(2) x(2)], [p(3) x(3)],'r-');
  plot3([p(1) y(1)], [p(2) y(2)], [p(3) y(3)],'g-');
  plot3([p(1) z(1)], [p(2) z(2)], [p(3) z(3)],'b-');
end

function R = Rx(a)
  R = [1       0       0;
       0  cos(a)  sin(a);
       0 -sin(a)  cos(a)];
end

function R = Ry(a)
  R = [cos(a) 0 -sin(a);
       0      1       0;
       sin(a) 0  cos(a)];
end

function R = Rz(a)
  R = [ cos(a) sin(a) 0;
       -sin(a) cos(a) 0;
            0       0 1];
end
