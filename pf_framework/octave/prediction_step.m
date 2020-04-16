function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);

for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  ur1_i = u.r1 - normrnd(0, r1Noise + transNoise);
  ut_i = u.t - normrnd(0, r1Noise + transNoise + r2Noise);
  ur2_i = u.r2 - normrnd(0, r2Noise + transNoise);
  
  delta = [ut_i * cos(particles(i).pose(3) + ur1_i);
           ut_i * sin(particles(i).pose(3) + ur1_i);
           ur1_i + ur2_i];
  
  particles(i).pose = particles(i).pose + delta;
  
  particles(i).pose(3) = normalize_angle(particles(i).pose(3));

end

end
