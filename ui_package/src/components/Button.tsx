import { styled, Button as mButton } from '@mui/material';
import { useEffect, useCallback, useRef, useState, SyntheticEvent } from "react";

import { 
  AttachTopic,
  DetachTopic,
  AdvertiseTopic,
  Transmit 
} from 'robot-ui-shared/lib/connection';

import {
  PublisherProps,
  SubscriberProps
} from 'robot-ui-shared';

const StyledButton = styled(mButton)({
  
  height: '70px',
  paddingRight: '10px',
  textAlign: 'center',
  margin: '20px'
});

type ButtonProps = PublisherProps & SubscriberProps & {
  direction?: string,
  data: string,
  color?: 'inherit' | 'primary' | 'secondary' | 'success' | 'error' | 'info' | 'warning';
  variant?: 'text' | 'outlined' | 'contained' | undefined,
  fullWidth?: boolean,
  disabled?: boolean,
  callback?: Function,
  sx?: any,
  children: any
}

export default function Button(props: ButtonProps) {
  const ref = useRef<HTMLButtonElement>(null);
  const [disable, setDisable] = useState(props.disabled);
  
  const listen = useCallback((data: any) => {
    if (!ref.current) return
    props.callback ? props.callback(ref.current, data) : null;
    setDisable(ref.current.disabled)
  }, []);

  useEffect(() => {
    AdvertiseTopic(props.advertiseName || 'button', props.advertiseType || 'std_msgs/String');
    if (props.topicName) {
      AttachTopic(props.topicName, props.topicType || 'std_msgs/Int32', listen);
      return () => DetachTopic(props.topicName, listen);
    }
  }, [listen, props]);

  function handleClick(e: SyntheticEvent) {
    Transmit(props.advertiseName || 'arrow', { data: props.data || '' });
  }
  
  return (
    <StyledButton 
      ref={ref} 
      variant={props.variant || "contained"} 
      color={props.color} 
      onClick={handleClick} 
      sx={{...props.sx }} 
      fullWidth={props.fullWidth}
      disabled={disable}>
      {props.children}
    </StyledButton>
  )
}