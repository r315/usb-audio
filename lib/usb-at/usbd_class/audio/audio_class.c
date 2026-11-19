/**
 **************************************************************************
 * @file     audio_class.c
 * @brief    usb audio class type
 **************************************************************************
 *                       Copyright notice & Disclaimer
 *
 * The software Board Support Package (BSP) that is made available to
 * download from Artery official website is the copyrighted work of Artery.
 * Artery authorizes customers to use, copy, and distribute the BSP
 * software and its related documentation for the purpose of design and
 * development in conjunction with Artery microcontrollers. Use of the
 * software is governed by this copyright notice and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
 * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
 * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
 * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
 * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
 *
 **************************************************************************
 */
#include "audio_class.h"
#include "audio_desc.h"
#include "usbd_core.h"
#include "audio.h"

static usb_audio_type audio_struct;

/**
 * @brief  Set current control attribute
 * 5.2.2.1.1 Set Terminal Control Request
 *
 * @param  udev: usb device core handler type
 * @param  setup: setup struct
 * @retval none
 */
static usb_sts_type audio_class_req_set_cur (usbd_core_type *pudev, usb_setup_type *setup, usb_audio_type *paudio)
{
    if (setup->wLength == 0){
        return USB_ERROR;
    }

    // Prepare to receive request data
    usbd_ctrl_recv (pudev, paudio->req_buf, setup->wLength);

    // Save which control is to be set
    switch (setup->bmRequestType & AUDIO_REQ_CONTROL_MASK){
        case AUDIO_REQ_CONTROL_INTERFACE:
            paudio->req_set_param = HBYTE(setup->wIndex);   // Interface (mic/spk)
            paudio->req_set_ctrl = HBYTE(setup->wValue);    // Control (vol, mute, freq)
        break;
        case AUDIO_REQ_CONTROL_ENDPOINT:
            paudio->req_set_param = setup->wIndex;          // Endpoint
            paudio->req_set_ctrl = AUDIO_FREQ_SET_CONTROL;
        break;
        default:
            return USB_NOT_SUPPORT;
    }

    paudio->req_set = AUDIO_REQ_SET_CUR;
    paudio->req_set_len = setup->wLength;

    // Control is set on ept0_rx when data is received
    return USB_OK;
}

/**
 * @brief Handle class specific request
 * See Audio device class, 5 Requests
 * @param pudev
 * @param setup
 * @return
 */
static usb_sts_type audio_class_request(usbd_core_type *pudev, usb_setup_type *setup)
{
    usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

    /* Fast path to SET_CUR (bRequest) */
    if (setup->bRequest == AUDIO_REQ_SET_CUR){
        return audio_class_req_set_cur (pudev, setup, paudio);
    }

    /* Check Terminal ID and Interface (wIndex) */
    switch(HBYTE (setup->wIndex)){
        /* Speaker Terminal ID */
        case AUDIO_SPK_FEATURE_UNIT_ID:
            /* Decode GET_CUR/MIN/MAX/RES (bRequest) */
            switch(setup->bRequest){
                case AUDIO_REQ_GET_CUR:
                    /* Check Control Selector (wValue) */
                    if (HBYTE (setup->wValue) == AUDIO_MUTE_CONTROL) {
                        paudio->req_buf[0] = audio_get_spk_mute();
                    } else {
                        *((uint16_t *) paudio->req_buf) = audio_get_spk_volume();
                    }
                break;

                case AUDIO_REQ_GET_MIN:
                    *((uint16_t *) paudio->req_buf) = paudio->spk.volume_limits[0];
                break;

                case AUDIO_REQ_GET_MAX:
                    *((uint16_t *) paudio->req_buf) = paudio->spk.volume_limits[1];
                break;

                case AUDIO_REQ_GET_RES:
                    *((uint16_t *) paudio->req_buf) = paudio->spk.volume_limits[2];
                break;

                default:
                    return USB_NOT_SUPPORT;
            }
        break;
        /* Mic Terminal ID */
        case AUDIO_MIC_FEATURE_UNIT_ID:
            /* Decode GET_CUR/MIN/MAX/RES (bRequest) */
            switch(setup->bRequest){
                case AUDIO_REQ_GET_CUR:
                    if (HBYTE (setup->wValue) == AUDIO_MUTE_CONTROL){
                        paudio->req_buf[0] = audio_get_mic_mute();
                    } else {
                        *((uint16_t *) paudio->req_buf) = audio_get_mic_volume();
                    }
                break;

                case AUDIO_REQ_GET_MIN:
                    *((uint16_t *) paudio->req_buf) = paudio->mic.volume_limits[0];
                break;

                case AUDIO_REQ_GET_MAX:
                    *((uint16_t *) paudio->req_buf) = paudio->mic.volume_limits[1];
                break;

                case AUDIO_REQ_GET_RES:
                    *((uint16_t *) paudio->req_buf) = paudio->mic.volume_limits[2];
                break;

                default:
                    return USB_NOT_SUPPORT;
            }
        break;
        default:
            return USB_NOT_SUPPORT;
    }

    usbd_ctrl_send (pudev, paudio->req_buf, setup->wLength);

    return USB_OK;
}


/**
 * @brief  Standard request set interface
 *
 * @param  udev: usb device core handler type
 * @param  setup: setup class
 * @retval none
 */
static void audio_std_req_set_interface (usbd_core_type *pudev, usb_setup_type *setup)
{
   usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

    #if AUDIO_SUPPORT_SPK
    if (LBYTE (setup->wIndex) == AUDIO_SPK_INTERFACE_NUMBER)
    {
        audio_set_spk_state (setup->wValue);
    }
    #endif

    #if AUDIO_SUPPORT_MIC
    if (LBYTE (setup->wIndex) == AUDIO_MIC_INTERFACE_NUMBER)
    {
        audio_set_mic_state (setup->wValue);

        if(setup->wValue){
            uint32_t len = audio_dequeue_data (paudio->mic.data) << 1;
            usbd_flush_tx_fifo (pudev, USBD_AUDIO_MIC_IN_EPT);
            usbd_ept_send (pudev, USBD_AUDIO_MIC_IN_EPT, (uint8_t*)paudio->mic.data, len);
        }
    }
    #endif

    usbd_ctrl_send_status (pudev);
}

/**
 * @brief  usb audio get interface
 * @param  udev: usb device core handler type
 * @param  setup: setup class
 * @retval none
 */
static void audio_std_req_get_interface (usbd_core_type *pudev, usb_setup_type *setup)
{
    usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

    #if AUDIO_SUPPORT_SPK
    if (LBYTE (setup->wIndex) == AUDIO_SPK_INTERFACE_NUMBER){
        paudio->req_buf[0] = audio_get_spk_state();
    }
    #endif

    #if AUDIO_SUPPORT_MIC
    if (LBYTE (setup->wIndex) == AUDIO_MIC_INTERFACE_NUMBER){
        paudio->req_buf[0] = audio_get_mic_state();
    }
    #endif

    usbd_ctrl_send (pudev, paudio->req_buf, 1);
}

/**
 * @brief  initialize usb custom hid endpoint
 * @param  udev: usb device core handler type
 * @retval status of usb_sts_type
 */
static usb_sts_type class_init_handler (void *udev)
{
   usb_sts_type status    = USB_OK;
   usbd_core_type *pudev  = (usbd_core_type *) udev;
   usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

   paudio->spk.volume_limits[0] = 0x0000; // MIN
   paudio->spk.volume_limits[1] = 0x1400; // MAX
   paudio->spk.volume_limits[2] = 0x003C; // RES 0.5dB step

   paudio->mic.volume_limits[0] = 0x0000; // MIN
   paudio->mic.volume_limits[1] = 0x1400; // MAX
   paudio->mic.volume_limits[2] = 0x003C; // RES 0.5dB step

#if AUDIO_SUPPORT_MIC
   /* open microphone in endpoint */
   usbd_ept_open (pudev, USBD_AUDIO_MIC_IN_EPT, EPT_ISO_TYPE,
                  AUDIO_MIC_IN_MAXPACKET_SIZE);
#endif

#if AUDIO_SUPPORT_SPK
   /* open speaker out endpoint */
   usbd_ept_open (pudev, USBD_AUDIO_SPK_OUT_EPT, EPT_ISO_TYPE,
                  AUDIO_SPK_OUT_MAXPACKET_SIZE);
#endif

#if AUDIO_SUPPORT_FEEDBACK
   /* open speaker feedback endpoint */
   usbd_ept_open (pudev, USBD_AUDIO_FEEDBACK_EPT, EPT_ISO_TYPE,
                  AUDIO_FEEDBACK_MAXPACKET_SIZE);
#endif

#if AUDIO_SUPPORT_SPK
   /* start receive speaker out data */
   usbd_ept_recv (pudev, USBD_AUDIO_SPK_OUT_EPT, (uint8_t*)paudio->spk.data,
                  AUDIO_SPK_OUT_MAXPACKET_SIZE);
#endif
   return status;
}

/**
 * @brief  clear endpoint or other state
 * @param  udev: usb device core handler type
 * @retval status of usb_sts_type
 */
static usb_sts_type class_clear_handler (void *udev)
{
    usb_sts_type status   = USB_OK;
    usbd_core_type *pudev = (usbd_core_type *) udev;

#if AUDIO_SUPPORT_MIC
    /* close in endpoint */
    usbd_ept_close (pudev, USBD_AUDIO_MIC_IN_EPT);
#endif

#if AUDIO_SUPPORT_SPK
    /* close out endpoint */
    usbd_ept_close (pudev, USBD_AUDIO_SPK_OUT_EPT);
#endif

#if AUDIO_SUPPORT_FEEDBACK
    /* close in endpoint */
    usbd_ept_close (pudev, USBD_AUDIO_FEEDBACK_EPT);
#endif

   return status;
}

/**
 * @brief  usb device class setup request handler
 * @param  udev: usb device core handler type
 * @param  setup: setup packet
 * @retval status of usb_sts_type
 */
static usb_sts_type class_setup_handler (void *udev, usb_setup_type *setup)
{
   usb_sts_type status   = USB_OK;
   usbd_core_type *pudev = (usbd_core_type *) udev;
   uint16_t len;

   switch (setup->bmRequestType & USB_REQ_TYPE_RESERVED){
        case USB_REQ_TYPE_CLASS:
            status = audio_class_request(pudev, setup);
            break;
        case USB_REQ_TYPE_STANDARD:
            /* Standard Request */
            switch (setup->bRequest){
                case USB_STD_REQ_GET_DESCRIPTOR:
                    if (HBYTE(setup->wValue) != AUDIO_DESCRIPTOR_TYPE){
                        status = USB_NOT_SUPPORT;
                    }else{
                        usbd_desc_t *config =
                            pudev->desc_handler->get_device_configuration ();
                        len = MIN (AUDIO_DESCRIPTOR_SIZE, setup->wLength);
                        usbd_ctrl_send (pudev, config->descriptor + 18, len);
                    }
                    break;
                case USB_STD_REQ_GET_INTERFACE: audio_std_req_get_interface (pudev, setup); break;
                case USB_STD_REQ_SET_INTERFACE: audio_std_req_set_interface (pudev, setup); break;
                case USB_STD_REQ_CLEAR_FEATURE: break;
                case USB_STD_REQ_SET_FEATURE: break;
                default: status = USB_NOT_SUPPORT; break;
            }
            break;
        /* Unsupported Requests */
        default: status = USB_NOT_SUPPORT; break;
    }

    if(status == USB_NOT_SUPPORT){
        usbd_ctrl_unsupport (pudev);
    }

    return status;
}

/**
 * @brief  usb device endpoint 0 in status stage complete
 * @param  udev: usb device core handler type
 * @retval status of usb_sts_type
 */
static usb_sts_type class_ept0_tx_handler (void *udev)
{
   usb_sts_type status = USB_OK;

   (void)udev;

   /* ...user code... */

   return status;
}

/**
 * @brief  usb device endpoint 0 out status stage complete
 * @param  udev: usb device core handler type
 * @retval status of usb_sts_type
 */
static usb_sts_type class_ept0_rx_handler (void *udev)
{
    usb_sts_type status    = USB_OK;
    usbd_core_type *pudev  = (usbd_core_type *) udev;
    usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

    /**
    * standard request set interface is called before this handler
    * to set which interface will be configured by control endpoint
    */
    if (paudio->req_set == AUDIO_REQ_SET_CUR) {
        /* class process */
        switch (paudio->req_set_ctrl){
            case AUDIO_VOLUME_CONTROL:
            {
                uint16_t volume = (uint16_t) (paudio->req_buf[0] |
                                             (paudio->req_buf[1] << 8));
                if (paudio->req_set_param == AUDIO_SPK_FEATURE_UNIT_ID) {
                    audio_set_spk_volume (volume * 100 /
                                            paudio->spk.volume_limits[1]);
                } else {
                    audio_set_mic_volume (volume * 100 /
                                            paudio->mic.volume_limits[1]);
                }
            }
            break;

            case AUDIO_MUTE_CONTROL:
                if (paudio->req_set_param == AUDIO_SPK_FEATURE_UNIT_ID) {
                    audio_set_spk_mute (paudio->req_buf[0]);
                } else {
                    audio_set_mic_mute (paudio->req_buf[0]);
                }
            break;

            case AUDIO_FREQ_SET_CONTROL:
            {
                uint32_t freq = (paudio->req_buf[0] |
                                (paudio->req_buf[1] << 8) |
                                (paudio->req_buf[2] << 16));
                if (paudio->req_set_param == USBD_AUDIO_MIC_IN_EPT) {
                    audio_set_mic_freq (freq);
                } else {
                    audio_set_spk_freq (freq);
                }
            }
            break;

            default:
                break;
        }
    }else{
        return USB_NOT_SUPPORT;
    }

    return status;
}

/**
 * @brief  usb device transmission complete handler
 * @param  udev: usb device core handler type
 * @param  ept_num: endpoint number
 * @retval status of usb_sts_type
 */
static usb_sts_type class_in_handler (void *udev, uint8_t ept_num)
{
   usb_sts_type status    = USB_OK;
   usbd_core_type *pudev  = (usbd_core_type *) udev;
   usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;
   uint32_t len           = 0;

   if ((ept_num & 0x7F) == (USBD_AUDIO_MIC_IN_EPT & 0x7F))
   {
      len = audio_dequeue_data (paudio->mic.data) << 1;
      usbd_flush_tx_fifo (udev, USBD_AUDIO_MIC_IN_EPT);
      usbd_ept_send (udev, USBD_AUDIO_MIC_IN_EPT, (uint8_t*)paudio->mic.data, len);
   }
   else if ((ept_num & 0x7F) == (USBD_AUDIO_FEEDBACK_EPT & 0x7F))
   {
      paudio->feedback_state = 0;
   }

   return status;
}

/**
 * @brief  usb device endpoint receive data
 * @param  udev: usb device core handler type
 * @param  ept_num: endpoint number
 * @retval status of usb_sts_type
 */
static usb_sts_type class_out_handler (void *udev, uint8_t ept_num)
{
   usb_sts_type status    = USB_OK;
   usbd_core_type *pudev  = (usbd_core_type *) udev;
   usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;
   uint16_t len;

   /* get endpoint receive data length  */
   len = usbd_get_recv_len (pudev, ept_num);

   if ((ept_num & 0x7F) == (USBD_AUDIO_SPK_OUT_EPT & 0x7F))
   {
      /* speaker data*/
      audio_enqueue_data (paudio->spk.data, len >> 1);
      /* get next data */
      usbd_ept_recv (pudev, USBD_AUDIO_SPK_OUT_EPT, (uint8_t*)paudio->spk.data,
                     AUDIO_SPK_OUT_MAXPACKET_SIZE);
      paudio->spk_out_stage = 1;
   }

   return status;
}

/**
 * @brief  usb device sof handler
 * @param  udev: usb device core handler type
 * @retval status of usb_sts_type
 */
static usb_sts_type class_sof_handler (void *udev)
{
   usb_sts_type status = USB_OK;
   (void)udev;
#if AUDIO_SUPPORT_FEEDBACK
   usbd_core_type *pudev  = (usbd_core_type *) udev;
   usb_audio_type *paudio = (usb_audio_type *) pudev->class_handler->pdata;

   if (paudio->spk_out_stage & 2)
   {
      paudio->spk_out_stage = 0;
   }
   else if (paudio->spk_out_stage)
   {
      paudio->spk_out_stage |= 2;

      if (0 == paudio->feedback_state)
      {
         usbd_core_type *pudev = (usbd_core_type *) udev;
         int len               = audio_get_spk_feedback (paudio->feedback);
         usbd_ept_send (pudev, USBD_AUDIO_FEEDBACK_EPT, paudio->feedback,
                        len);
         paudio->feedback_state = 1;
      }
      if (paudio->feedback_state++ >
          (1 << FEEDBACK_REFRESH_TIME))   // timeout
      {
         usbd_core_type *pudev = (usbd_core_type *) udev;
         int len               = audio_get_spk_feedback (paudio->feedback);
         usbd_flush_tx_fifo (pudev, USBD_AUDIO_FEEDBACK_EPT);
         usbd_ept_send (pudev, USBD_AUDIO_FEEDBACK_EPT, paudio->feedback,
                        len);
         paudio->feedback_state = 1;
      }
   }
#endif

   return status;
}

/**
 * @brief  usb device event handler
 * @param  udev: usb device core handler type
 * @param  event: usb device event
 * @retval status of usb_sts_type
 */
static usb_sts_type class_event_handler (void *udev, usbd_event_type event)
{
   usb_sts_type status    = USB_OK;
   (void)udev;

   switch (event)
   {
      case USBD_RESET_EVENT:
         break;
      case USBD_SUSPEND_EVENT:
         audio_suspend_event ();
         break;
      case USBD_WAKEUP_EVENT:
         break;
      case USBD_INISOINCOM_EVENT:
         break;
      default:
         break;
   }
   return status;
}

/* usb device class handler */
usbd_class_handler audio_class_handler = {
    class_init_handler,    class_clear_handler,   class_setup_handler,
    class_ept0_tx_handler, class_ept0_rx_handler, class_in_handler,
    class_out_handler,     class_sof_handler,     class_event_handler,
    &audio_struct};
