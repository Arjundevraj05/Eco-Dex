'use client';

import NextTopLoader from 'nextjs-toploader';

export default function TopLoader() {
  return (
    <NextTopLoader
      color="#16a34a"
      height={3}
      showSpinner={false}
      crawlSpeed={200}
      speed={400}
    />
  );
}
