### Problems in my first-pass version && Reflection
  
  1. no 'main()' function, so the file does nothing when I run it.
     I forgot to add the 'main()' function in the first pass. Then, I rechecked the source file and discovered that I indeed forgot to write this part. Then, I added the main function at the bottom of the file.

  2. '--Kd 0' was rejected, so I couldn't run pure P control with this tool
     In my first version, Kd <= 0 raised an error. But Kd = 0 is the P controller, which is the baseline my need to compare against. I read the stretch goal again and rechecked the corresponding function: checking whether the value of Kd is valid; Then, I modified codes in this part.

  3. The default Kd = 0.1 is too small to show any effect.
     The useful size of Kd depends on the joint's inertia. The shoulder's effective inertia is about 0.073 kg·m². A rough guide for the Kd that just stops overshoot is 2 × √(Kp × I), which is about 1.2 at Kp = 5 and 3.8 at Kp = 50. So 0.1 is less than a tenth of what's needed, and at Kp = 50 it cut the overshoot by only 3%. A P-vs-PD plot with that default would show almost no difference. Then, I modified the default value of Kd (damping) to 0 if I want to reset the value I will input in the command.

### What I learned
    I learned some physic knowledge
      the formula that computes the value of Kd that just stops overshoot: 2 × √(Kp × I)