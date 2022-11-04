# QCR PLATFORM_ID__PASCAL User Interface
This is a [Next.js](https://nextjs.org/) project bootstrapped with [`create-next-app`](https://github.com/vercel/next.js/tree/canary/packages/create-next-app). Package generated using the QCR Tools *[Code Templates](https://github.com/qcr/code_templates)*

## Getting Started for the First Time

If this is the first time using this User Interface (UI), please do the following:
1. Make sure you have NodeJs and NPM installed (latest version) - see [here](https://qcr-docs.qut.edu.au/guides/installing_software/nodejs/) for details.
2. Run the npm install and fix any vulnerabilities
```bash
# Installs your User Interface's required packages
npm install

# Fix any vulnerabilities with the following
npm audit fix
```

## Adding your UI to a QCR Git Repository
In order to add your UI package to the QCR git repository, simply:
1. Create a repository in the qcr github organisation (private is preferable) matching the name of your project (root name of this package)
2. Follow the below steps to initialise and push your new repo (make sure the URL is correct when adding as your remote)
```bash
# Init git
git init

# Add the template contents
git add .

# Commit your work
git commit -m "first UI upload"

# Rename branch to main, add remote and push
git branch -M main
git remote add origin https://github.com/qcr/PACKAGE_NAME.git
git push -u origin main
```

## Developing your User Interface
You can run the development server locally to test how your UI looks and functions. Running locally means this will allow you to open [http://localhost:3000](http://localhost:3000) with your browser to see the result:

```bash
# Runs on http://localhost:3000
npm run dev
```

You can start editing the page by modifying `pages/index.tsx`. The page auto-updates as you edit the file. Included are some helpful comments within this file to get you started. Remember to get the [robot-ui-server](https://github.com/qcr/robot-ui-server) into a ROS workspace and follow the README instructions for tailoring it to your own UI.

TEMPLATE_START ADD_CUSTOM_LOGO
### Adding a Custom Logo to your UI
In some cases, say for industry demonstrations, you may wish to include other affiliates and their respective logo(s). To do so, simply:
1. Add your logo (preferably as an svg or png) to the *public* folder
2. Edit the /src/components/TopBar.tsx file to include your logo's name in the PLATFORM_ID__PASCALContainer located near the bottom. An example is already provided with the custom_logo.svg

TEMPLATE_END
## Deploy your User Interface
***NOTE: you will need QCR admin access for these steps - please refer to [QCR Request Technical Support](https://qcr-docs.qut.edu.au/guides/researcher_support/requesting_support/) for any assistance***

When you are happy with the result of your UI and its functionality, you can *deploy* this to our qcr AWS instance, allowing for remote connection of your UI to the robot. This is achieved by:
1. Run the following command to build your project into an out folder
```bash
npm run build
```
2. Once completed, copy the contents of the out folder to the AWS server to be uploaded with the following command:
```bash
rsync -r --progress out/ ubuntu@qcr.ai:~/platforms/PLATFORM_ID
```
Once released/built, your platform can be accessed via the [QCR Platforms](https://platforms.qcr.ai/).

## Learn More
To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js/) - your feedback and contributions are welcome!