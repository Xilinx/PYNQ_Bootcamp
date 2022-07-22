<!-- vim: tw=80:cc=80:spell:nowrap
-->

# Contributing

If you are an intern who has improved a bootcamp module/brakeout session, we
emplore you to contribute your changes upstream for continued improvement of the
materials we have so far.

## Quick Guide to opening a Pull Request

### Cloning your Fork

- (1) Fork the repo from PYNQ_bootcamp  on your github account (url:
   https://github.com/<your-github-username>/PYNQ_Bootcamp)

- (2) Clone the fork to your PYNQ via the web terminal (New -> Terminal)
   ```bash
   cd ~/jupyter_notebooks
   git clone https://github.com/<your-github-username>/PYNQ_Bootcamp
   cd PYNQ_Bootcamp
   ```

- (3) Make changes to your work tree as needed
- (4) `git add .` (will add all the changes)
- (5) `git status` (to check if the changes are staged)
- (6) `git commit -m "comment"` (invoking the command without the m flag with open
   your editor for editing the commit)
- (7) `git push --set-upstream origin master` (push the changes to your forked repo
   and set the default upstream push branch default to `origin master`)

### Submitting a Pull request

- (1) Now on the git forked repo click the 3 option - pull request
- (2) Next, click on the green button on the right side "new pull request"
- (3) under the Compare changes click on the blue link "compare across forks"
- (4) select the your push repo under "head repository" drop-down list.
- (5) last, hit the "create pull request". (edited)
