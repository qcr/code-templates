import { styled, Grid } from '@mui/material';

import {
  QcrPage,
} from 'qcr-sites-shared';

import {
  MediaStreamViewer,
  BottomBar,
  Platform
} from 'robot-ui-shared';

// If you make your own custom components, remember to include them here
// The path they are referenced from is /src/components/<your-custom-component>
// You will also need to update the /src/index.ts with your new component path
import {
  Button,
  TopBar
} from 'PLATFORM_ID-platform'

const TABS = [
  {text: 'Home', target: 'https://platforms.qcr.ai/home/'},
  {text: 'Logout', target: 'https://platforms.qcr.ai/logout'},
];

// You can modify this for any type of text input within the UI
// Default Example: 
//    used for the 'Controls' heading, where you 
//    would put buttons or other forms of control
//    for your application
const ControlHeading = styled('div')({
  color: 'white', 
  padding: '5px',
  fontFamily: 'Roboto,-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Oxygen,Ubuntu,Cantarell,Fira Sans,Droid Sans,Helvetica Neue,sans-serif',
});

// This callback example enables or disables an element (in this case a button)
function buttonCb(element: HTMLButtonElement, data: any) {
  if (data.data) {
    element.disabled = false;
  } else {
    element.disabled = true;
  }
}

// The main page created by default - please modify this to your liking
// A few points to make things easier:
//    - A Grid has default break points (e.g. xs) with a max of 12 available columns 
//      See here (https://mui.com/system/react-grid/) for details
//    - The MediaStreamViewer can host different views from your package - remember to update the topic name
//    - The included Button component will advertise (publish) on the given topic name (feel free to change)
//      Remember to also specify the message type of the topic in advertiseType
//      The topicName variable can be used to enable a GUI element - the example enables the execute and cancel buttons when your 
//      package is ready
function Page() {
  return (
    <QcrPage sx={{backgroundColor: 'black'}}>
      <TopBar title="PLATFORM_ID__PASCAL Demonstrator" tabs={TABS} selected={0} />
      <Grid container sx={{
        // alignItems: 'center'
      }}>
        {/* <Grid item xs={0.3}></Grid> */}
        <Grid item xs={0.5}></Grid>
        <Grid item xs={7} sx={{ height: 'calc(100vh - 92px - 48px)', position: 'relative' }}>
          <MediaStreamViewer title="Primary Camera" topicName='/camera_1/color/image_raw' />
        </Grid>
        <Grid item xs={1}></Grid>
        <Grid item xs={3}>
          <Grid container>
            <Grid item xs={12}>
              <MediaStreamViewer title="Secondary Camera" topicName='/camera_2/color/image_raw' />
            </Grid>
            <Grid item xs={12}>
              <ControlHeading>Controls</ControlHeading>
              <Button  data="0" advertiseName='/PLATFORM_ID/ui/button' advertiseType='std_msgs/Int32' topicName='' fullWidth>
                Example Button 0
              </Button>
              <Button  data="1" advertiseName='/PLATFORM_ID/ui/button' advertiseType='std_msgs/Int32' topicName='' fullWidth>
                Example Button 1
              </Button>
              <Grid container>
                <Grid item xs={5}>
                  <Button  data="1" advertiseName='/PLATFORM_ID/ui/execute' advertiseType='std_msgs/Int32' topicName='/PLATFORM_ID/ui/ready' topicType='std_msgs/Bool' variant='outlined' callback={buttonCb} sx={{
                    background: 'white',
                      '&:hover': {
                          backgroundColor: '#ddd',
                      },
                    }} fullWidth disabled>
                    Execute
                  </Button>
                </Grid>
                <Grid item xs={2} />
                <Grid item xs={5}>
                  <Button  data="0" advertiseName='/PLATFORM_ID/ui/execute' advertiseType='std_msgs/Int32' topicName='/PLATFORM_ID/ui/ready' topicType='std_msgs/Bool' variant='outlined' callback={buttonCb} sx={{
                    background: 'white',
                      '&:hover': {
                          backgroundColor: '#ddd',
                        }
                    }} fullWidth disabled>
                    Cancel
                  </Button>
                </Grid>
              </Grid>
            </Grid>
          </Grid>
        </Grid>
        <Grid item xs={0.5} />
      </Grid>
      <BottomBar />
      <Platform remoteId={process.env.platformId || ''} />
    </QcrPage>
  )
}

export default Page
