const path = require('path');
const { PHASE_DEVELOPMENT_SERVER } = require('next/constants')

/** @type {import('next').NextConfig} */
module.exports = (phase, { defaultConfig }) => {
  let configs = {
    reactStrictMode: true,
    extends: [
      'plugin:@next/next/recommended',
    ],
    webpack: (config) => {
      config.resolve.alias['PLATFORM_ID-platform'] = path.resolve(__dirname, 'src/');
      config.experiments.topLevelAwait = true;
      return config;
    },
    env: { platformId: 'PLATFORM_ID' }
  }
  
  if (phase === PHASE_DEVELOPMENT_SERVER) {
    configs.env = Object.assign(
      configs.env, {
        user_token: 'gGWRAKmcc8Xz3XVpCg59v2',
        user_key: 'hPHcBZ3MiMaABzB34N3qPc'
      }
    )
  } else {
    configs = Object.assign(
      configs, { basePath: '/platform/PLATFORM_ID' }
    )
  }
  
  return configs;
};
