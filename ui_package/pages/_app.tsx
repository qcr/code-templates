import type {AppProps} from 'next/app';
import Head from 'next/head';

import {ThemeProvider} from '@mui/material/styles';

import {QcrFavicon, qcrTheme} from 'qcr-sites-shared';
import { ToastContainer } from 'react-toastify';

import 'qcr-sites-shared/lib/styles/styles.css';
import 'react-toastify/dist/ReactToastify.css'; 

const theme = qcrTheme();

function MyApp({ Component, pageProps }: AppProps) {
  return (
    <>
      <QcrFavicon />
      <Head>
        <title>PLATFORM_ID__PASCAL Demonstrator</title>
      </Head>
      <ThemeProvider theme={theme}>
        <Component {...pageProps} />
        <ToastContainer
          position="bottom-center"
          autoClose={3000}
          hideProgressBar={false}
          newestOnTop={false}
          closeOnClick
          rtl={false}
          pauseOnFocusLoss
          draggable
          pauseOnHover
          theme="dark"
        />
      </ThemeProvider>
    </>
  );
}

export default MyApp
