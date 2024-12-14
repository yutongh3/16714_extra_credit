# 16714_extra_credit
Same .m codes are in /scripts

Video for code examples:

LQR:
https://youtu.be/emXJfF-Ou2w

MPC:
https://youtu.be/GZaHwXqBMV8

LQG:
https://youtu.be/-kz9NUTAU_8



Questions:
1.1: Q and R are cost term for states and control.\\
     With lager R, the control will be less aggresive.\\
     Larger Q_xx will make the ee go the the goal position faster. (smaller (x-xT) cost)\\
     Larger Q_vv will make the system travel slower. (smaller (v-vT) cost)\\
\\
1.2: control constraint is applied on the acceleration and will make the the \\
     tarjectory have a lager radius.\\

1.3: Velocity constraint will make the system travel slower.
\\
1.4: Q and R are the same as LQR problem.\\
     W is the covariance of the gaussian white noise acting on the dynamic,\\
     This will control unreliable.\\
     V is the covariance of the gaussian white noise acting on the mesurement.\\
     With lager V or W, the system trajectory become more random and follows\\
     the original LQR trajectory less accurate.\\