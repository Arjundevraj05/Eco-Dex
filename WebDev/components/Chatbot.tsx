// components/Chatbot.tsx
'use client'
import React, { useEffect } from 'react';

const Chatbot: React.FC = () => {
  useEffect(() => {
    // This script will be added to the page when the component mounts
    const script = document.createElement('script');
    script.src = 'https://www.chatbase.co/embed.min.js';
    script.async = true;
    script.setAttribute('chatbotId', 'QKscKMNPc1acEGjakNcTZ');
    script.setAttribute('domain', 'www.chatbase.co');
    document.body.appendChild(script);
    
    // Cleanup the script when the component unmounts
    return () => {
      document.body.removeChild(script);
    };
  }, []);

  return (
    <div>
      <iframe
        src="https://www.chatbase.co/chatbot-iframe/QKscKMNPc1acEGjakNcTZ"
        width="100%"
        style={{ height: '100%', minHeight: '700px' }}
        frameBorder="0"
      ></iframe>
    </div>
  );
};

export default Chatbot;
