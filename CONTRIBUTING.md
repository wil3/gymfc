# Contributing to GymFC

:+1::tada: Thank you for wanting to contribute to GymFC! :+1::tada: 

Together we can build next
generation flight control systems :rocket: :airplane: :helicopter: and there are many ways you can help! 

## Bug reporting
Please follow these steps when reporting a bug.

1) Search through the issues to ensure your bug has not already been submitted. 
2) Open a new issue and describe at lengths how the bug can be reproduced.
3) Provide details of your running environment, the commands you have executed
and the full output. If necessary, provide any material or scripts needed to
reproduce the bug. 

## Code Contributions
Please following these steps in order to have your contribution considered by
the maintainers.

1) Search through the issues to verify your code changes have not already been
discussed. 
2) Open an issue on Github outlining the changes you would like to make. Scope
each issue to a single change. If there are multiple changes you'd like to
make, open an issue for each one. The issue serves as the location for
documenting the change to be made. There will exist a one-to-one mapping between
an issue and a pull request. Once the change is approved continue to the
next step.
2) If you haven't already done so, fork the gymfc repository.
2) Create a branch named `i[issue number]-brief-description` where `[issue
number]` is the issue number previously created and `brief-description` is a
brief description of the change you are making. For example `i45-contribution`.

3) Make changes on your new branch. Follow the projects coding style and
 test you code changes. Double check your commits to ensure
unnecessary commits are not being to keep the commit history clean. 

4) When you are happy with the code changes open up a pull request. Specify the
issue the PR closes, for example `Closes: #45`. When the PR is merged, this keyword will automatically close the corresponding issue. 
Please provide testing results that will affect your code changes. A number of
testing scripts already exist in /test that may help. 

## User Module Development

TODO 

## Coding Style

We adhere to Google's style guide for Python code.
The Python guide can be found
[here](http://google.github.io/styleguide/pyguide.html). 

To stay consistent with Gazebo, the Gazebo C++ plugins follow the Gazebo style guide found
[here](http://gazebosim.org/tutorials?tut=contrib_code&cat=development).

